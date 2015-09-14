#!/usr/bin/env bash
#scp stab-1-splits2/0-1-0.avi anttrack@vatic2.stanford.edu:/vatic2/uavdata2/videos/
scp stab-1-splits2/1-0-0.avi stab-1-splits2/1-1-0.avi stab-1-splits2/1-2-0.avi stab-1-splits2/2-1-0.avi anttrack@vatic2.stanford.edu:/vatic2/uavdata2/videos/
#scp stab-1-splits2/1-1-0.avi anttrack@vatic2.stanford.edu:/vatic2/uavdata2/videos/
#scp stab-1-splits2/1-2-0.avi anttrack@vatic2.stanford.edu:/vatic2/uavdata2/videos/
#scp stab-1-splits2/2-1-0.avi anttrack@vatic2.stanford.edu:/vatic2/uavdata2/videos/

turkic extract ../uavdata2/videos/0-1-0.avi ../uavdata2/raw/0-1-0/ --no-resize