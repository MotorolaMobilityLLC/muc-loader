#!/bin/bash

defconfig_list=$(find configs -iname defconfig)

for cfg in $defconfig_list; do
  configpath=$(dirname "$cfg")
  mod=$(echo "$configpath" | sed -e "s:^configs/::")

  echo ""
  echo "============================================"
  echo "== " $mod
  echo "============================================"
  echo ""

  if ! bash ./configure ${mod}; then
      printf '%s failed!' "configure ${mod}" >&2
      exit 1
  fi

  make
  if test $? -ne 0; then
      printf '%s failed!' "make ${mod}" >&2
      exit 1
  fi

  make distclean

done
