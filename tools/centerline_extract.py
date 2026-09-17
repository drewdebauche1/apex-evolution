#!/usr/bin/env python3
"""
Extract a centerline from a top-down track image and output waypoints JSON.
Usage:
  python centerline_extract.py --in track.png --out waypoints.json
  python centerline_extract.py --test   # generates a synthetic track and runs pipeline
"""
import argparse
import json
import cv2
import numpy as np
from skimage.morphology import skeletonize


def extract_centerline(img):
    # img: grayscale numpy array
    # preprocess
    blur = cv2.GaussianBlur(img, (5,5), 0)
    _,thr = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # invert if track is dark on light
    if np.mean(img[thr==255]) < np.mean(img[thr==0]):
        thr = 255 - thr
    # morphological closing to fill gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
    closed = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernel)
    # pick largest connected component
    num,labels = cv2.connectedComponents(closed)
    if num <= 1:
        mask = closed>0
    else:
        areas = [(labels==i).sum() for i in range(1,num)]
        best = 1 + int(np.argmax(areas))
        mask = (labels==best)
    # skeletonize expects boolean image where foreground==True
    ske = skeletonize(mask)
    ys, xs = np.where(ske)
    if len(xs) == 0:
        raise RuntimeError("No skeleton found; try a different threshold or input image")
    pts = np.column_stack([xs.astype(float), ys.astype(float)])
    # order points by greedy nearest neighbor from leftmost
    start_idx = int(np.argmin(pts[:,0]))
    ordered = [pts[start_idx]]
    used = set([start_idx])
    cur = start_idx
    for _ in range(len(pts)-1):
        d = np.linalg.norm(pts - pts[cur], axis=1)
        d[list(used)] = 1e9
        nxt = int(np.argmin(d))
        ordered.append(pts[nxt])
        used.add(nxt)
        cur = nxt
    ordered = np.array(ordered)
    # resample to N points along polyline
    seg = np.sqrt(((ordered[1:]-ordered[:-1])**2).sum(axis=1))
    cum = np.concatenate([[0], np.cumsum(seg)])
    total = cum[-1]
    if total == 0:
        return ordered.tolist()
    N = min(300, max(50, int(total//2)))
    dist_samples = np.linspace(0,total,N)
    res = []
    idx = 0
    for d in dist_samples:
        while idx < len(cum)-1 and cum[idx+1] < d:
            idx += 1
        if idx >= len(cum)-1:
            res.append(ordered[-1].tolist())
        else:
            t = (d - cum[idx]) / (cum[idx+1]-cum[idx]+1e-12)
            p = ordered[idx] * (1-t) + ordered[idx+1] * t
            res.append(p.tolist())
    return res


def generate_synthetic(path):
    W,H = 800,600
    img = np.full((H,W,3),255,dtype=np.uint8)
    # draw a curving thick track
    pts = np.array([[100,300],[300,200],[500,200],[700,300],[600,450],[400,450],[200,350]])
    cv2.fillPoly(img, [pts+np.array([0,0])], (100,100,100))
    # draw inner hole to make it ring-like
    scale = 0.6
    center = pts.mean(axis=0)
    inner = ((pts-center)*scale + center).astype(int)
    cv2.fillPoly(img, [inner], (255,255,255))
    cv2.imwrite(path, img)
    return path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="infile", help="input image path")
    p.add_argument("--out", dest="outfile", help="output json path")
    p.add_argument("--test", action="store_true")
    args = p.parse_args()
    if args.test:
        imgp = 'tools/synthetic_track.png'
        generate_synthetic(imgp)
        infile = imgp
        outfile = 'tools/synthetic_waypoints.json'
    else:
        infile = args.infile
        outfile = args.out or 'waypoints.json'
    img = cv2.imread(infile, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to load {infile}")
    pts = extract_centerline(img)
    with open(outfile,'w') as f:
        json.dump({'points': [[float(x), float(y)] for x,y in pts]}, f, indent=2)
    print('Wrote', outfile)

if __name__ == '__main__':
    main()
