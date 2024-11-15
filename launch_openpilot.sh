#!/usr/bin/bash
export ATHENA_HOST='ws://mr-one.cn:7899'
export API_HOST='http://mr-one.cn:7898'
export MAPBOX_TOKEN='pk.eyJ1IjoibXJvbmVjYyIsImEiOiJjbHhqbzlkbTYxNXUwMmtzZjdoMGtrZnVvIn0.SC7GNLtMFUGDgC2bAZcKzg'
export SKIP_FW_QUERY=1
exec ./launch_chffrplus.sh
