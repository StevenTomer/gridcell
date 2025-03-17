#!/bin/bash

build/gridcell tests/unit_tests.cfg 2>&1 | tee tests/unit_tests.lastoutput
