# Apex Evolution

Version 1.5 of the local browser game.

Apex Evolution is a small neural-network racing lab: draw or select a track, train 50 racers at a time, and watch neuroevolution search for the fastest legal route from start to finish.

## Run on Windows

1. Extract this ZIP using Extract All.
2. Open the extracted apex-evolution folder in your code editor or a terminal.
3. With Python 3 installed, run: `py serve.py`
4. Open http://localhost:8000 in your browser.
5. Leave the terminal open while playing; press Ctrl+C to stop the server.

On macOS/Linux use `python3 serve.py` instead. If Windows does not recognize `py` but Python is installed, try `python serve.py`.

Do not double-click index.html: the JavaScript modules need a local HTTP server.
No npm installation, build step, API key, or external service is required.

## Files

- index.html: page structure and controls
- style.css: appearance and responsive layout
- app.mjs: interface, rendering, keyboard controls, track slots, saves, and training loop
- engine.mjs: car physics, track geometry, neural-network inference, and neuroevolution
- serve.py: local-only development server (Python standard library)

## Current Behavior

50 neural-network racers train per generation; five randomly selected racers are shown. AI and manual driving share the selected numbered track. Press W or Up to start a manual run. Car speed is adjustable from 25 to 300 px/s while running.

Each racer has a feed-forward neural network with 10 sensor inputs, 8 hidden neurons, and 2 outputs that directly control steering and throttle. The 106 weights and biases are trained with a genetic algorithm: three elites survive, offspring inherit and mutate successful networks, and five new networks maintain diversity. This is neuroevolution rather than backpropagation-based reinforcement learning.

The yellow boundary is the limit for the car's white center dot. A forward crossing of the usable finish line stops the timer. The champion path and replay are recorded locally.

## Braking and steering

Down arrow or S applies the brakes. Braking is stronger than acceleration: from 125 px/s, a full stop takes about 0.37 seconds; from 300 px/s, about 0.88 seconds. Steering builds and returns smoothly, cannot rotate the car at rest, and is limited by tire grip at higher speeds. Brake before tight turns.

AI drivers use the same physics. They search the drivable road surface for the shortest legal guide route from the start to the finish, then brake for the guide route's corners instead of blindly honoring every center-line bend. Boundary and finish checks still use the actual road and finish line. If a shortcut is legal but too sharp to hold at speed, braking and steering limits decide whether the car can make it. Finishing quickly remains the reward; touching every drawn corner earns no extra points.

Existing tracks, results, and champion replays remain available. Version 1.3 driving parameters are automatically converted to a valid baseline neural network when loaded. Previously recorded champion times and replays may reflect the older controller. Use **Reset this course** if you want a fresh comparison.

Run the physics checks with `node --test app.test.mjs engine.test.mjs` (Node.js required only for testing).

## Add more tracks

Click **Add track** beside the saved-track selector. Click or tap the canvas to draw a course, then click **Save track**. An unused track number is selected automatically (up to 999 saved tracks). Your new course appears in the selector and has its own training progress and champion. Use **Edit track** to change an existing course.

## Saved progress

Tracks, trained drivers, and champions use browser localStorage under `apex-evolution-v1`. The hosted website and localhost have separate browser storage: downloading the source does not include or automatically transfer your existing custom tracks or training progress. Keep the same local hostname and port when you want to reuse local saves.

## Editing

Edit the files and refresh the browser. This download is independent of the hosted website; local edits do not update the online game. The archive excludes hosting credentials, Git history, and deployment configuration.

## Version 1.5

- AI and manual driving now use the same strict track-boundary rule; tiny AI wall-riding corrections were removed
- Recalibrated neural baseline and safer initial population variation

## Version 1.4

- Replaced the hand-tuned driving policy with a 10 → 8 → 2 feed-forward neural network
- Neural-network weights and biases evolve locally across generations
- Existing saved tracks and champion replays remain compatible

## Version 1.3

- Rounded track rendering and collision checks use the same geometry
- Corner-radius edits save correctly, preserve zero, and cancel without changing the track
- Sector timing and personal-best comparisons
- Stronger deceleration when coasting

## Version 1.0.0

This release marks the first stable local version:

- Custom saved tracks, with **Add track** support up to 999 numbered tracks
- Adjustable car speed and stronger braking
- Shared manual and AI car physics
- Fastest-route AI guide planning through the legal road surface
- Per-track saved champions, populations, and training history
- Track diagnostics export for debugging custom courses
