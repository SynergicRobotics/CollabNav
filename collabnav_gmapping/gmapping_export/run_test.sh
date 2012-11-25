#!/bin/bash

# one_test = bin/gfs_nogui -filename data/nsh_level_a.log

function run_N_times {
  for i in {1..10}
  do
    bin/gfs_nogui -filename data/nsh_level_a.log 2>>output.txt | grep -v 'filename'
  done
}

#for i in 1 2 4 8 12 16 20 24
for i in {1..24}
do
  export OMP_NUM_THREADS=$i
  echo -n "time_threads_$i = [ "
  run_N_times
  echo "];"
done
