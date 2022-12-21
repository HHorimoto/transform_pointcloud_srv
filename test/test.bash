#!/bin/bash

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

# Run test #
timeout 10 ros2 launch transform_pointcloud_srv test_launch.py > test.log

## Test ##
grep "Run test succeeded" test.log