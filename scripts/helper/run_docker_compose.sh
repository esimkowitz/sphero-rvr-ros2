#!/bin/bash

# start aliases for pushd/popd so they don't echo to console
pushd () {
    command pushd "$@" > /dev/null
}

popd () {
    command popd "$@" > /dev/null
}
# end pushd/popd aliases

# By default, we do not want to mock the RVR interface or build the container image
mock_rvr_val="false"
shouldbuild=false

# Start process option flags if present
while getopts "bm" flag
do
    case "${flag}" in
        b) 
            echo "Running docker compose with optional build flag"
            shouldbuild=true
            ;;
        m) 
            echo "Mocking the RVR interface"
            mock_rvr_val="true"
            ;;
    esac
done
# End process option flags

# Get the root directory for the git repo
reporoot=$(git rev-parse --show-toplevel)

# start docker compose, should run from the repo root directory, then return to the pwd on exit
pushd $reporoot
if $shouldbuild; then
    trap "MOCK_RVR=$mock_rvr_val docker compose up --build" EXIT
else
    trap "MOCK_RVR=$mock_rvr_val docker compose up" EXIT
fi
popd
# end docker compose