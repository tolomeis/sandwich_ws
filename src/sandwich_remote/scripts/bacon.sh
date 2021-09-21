#!/bin/bash
cd $(rospack find sandwich_remote)/scripts/GUI
python3 -m http.server 8080 --bind $(hostname -I | awk '{print $1;}')
