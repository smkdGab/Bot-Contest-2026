import os

# Tests exercise pygame surfaces without opening a desktop window. The separate
# Xvfb dashboard check uses the real display path.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
