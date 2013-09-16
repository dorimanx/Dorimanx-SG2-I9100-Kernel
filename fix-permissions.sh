#!/bin/bash

find . -type f -name '*.h' -exec chmod 644 {} \;

find . -type f -name '*.c' -exec chmod 644 {} \;

find . -type f -name '*.py' -exec chmod 755 {} \;

find . -type f -name '*.sh' -exec chmod 755 {} \;

find . -type f -name '*.pl' -exec chmod 755 {} \;
