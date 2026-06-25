#!/usr/bin/env bash
echo -n "2" > /data/params/d/HasAcceptedTerms
echo -n "0.2.0" > /data/params/d/CompletedTrainingVersion
echo -n "1" > /data/params/d/IsMetric
export API_HOST=https://api.konik.ai
export ATHENA_HOST=wss://athena.konik.ai
#export MAPS_HOST=https://api.konik.ai/maps
export MAPBOX_TOKEN='pk.eyJ1IjoibXJvbmVjYyIsImEiOiJjbHhqbzlkbTYxNXUwMmtzZjdoMGtrZnVvIn0.SC7GNLtMFUGDgC2bAZcKzg'

exec ./launch_chffrplus.sh
