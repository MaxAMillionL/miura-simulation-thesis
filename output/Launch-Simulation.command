#!/bin/zsh
cd -- "${0:A:h}" || exit 1
exec .venv/bin/mjpython -m miura_robot.run --paused --seconds 3600
