#!/bin/bash

splice_videos() {
    local vid0;
    vid0="$1";
    local vid1;
    vid1="$2";
    local vid2;
    vid2="$3";
    local output;
    output="$4";
    local filter;
    filter=$(cat <<EOF
[0:v]crop=256:ih:192:0[v0];
[1:v]crop=256:ih:192:0[v1];
[2:v]crop=256:ih:192:0[v2];
[v0][v1][v2]hstack=inputs=3
EOF
    )

    local rng;
    rng=$((RANDOM % 6));

    if [ $rng == 0 ]; then
        ffmpeg -i "$vid0" -i "$vid1" -i "$vid2" -filter_complex "$filter" "$output"
        echo "order is: $vid0 - $vid1 - $vid2"
    elif [ $rng == 1 ]; then
        ffmpeg -i "$vid0" -i "$vid2" -i "$vid1" -filter_complex "$filter" "$output"
        echo "order is: $vid0 - $vid2 - $vid1"
    elif [ $rng == 2 ]; then
        ffmpeg -i "$vid1" -i "$vid0" -i "$vid2" -filter_complex "$filter" "$output"
        echo "order is: $vid1 - $vid0 - $vid2"
    elif [ $rng == 3 ]; then
        ffmpeg -i "$vid1" -i "$vid2" -i "$vid0" -filter_complex "$filter" "$output"
        echo "order is: $vid1 - $vid2 - $vid0"
    elif [ $rng == 4 ]; then
        ffmpeg -i "$vid2" -i "$vid0" -i "$vid1" -filter_complex "$filter" "$output"
        echo "order is: $vid2 - $vid0 - $vid1"
    elif [ $rng == 5 ]; then
        ffmpeg -i "$vid2" -i "$vid1" -i "$vid0" -filter_complex "$filter" "$output"
        echo "order is: $vid2 - $vid1 - $vid0"
    fi
}

splice_videos "$1" "$2" "$3" "$4"
