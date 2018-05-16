#!/usr/bin/env bats

setup() {
    if [ ! -d ./expanded ];then mkdir expanded ;fi
}

@test "All paths are valid in astar" {
    if [ ! -f gastar ];then
        bash ./run -full map-for-test/ca_cave.map gastar > gastar
    fi
    cat astar | grep subopt | awk '{if($14 == "invalid") exit 1;}'
    [ "$?" -eq 0 ]
}

@test "All paths are optimal in astar" {
    if [ ! -f gastar ];then
        bash ./run -full map-for-test/ca_cave.map gastar > gastar
    fi
    cat astar | grep subopt | awk '{if($11 < 0.99999) exit 1;}'
    [ "$?" -eq 0 ]
}
