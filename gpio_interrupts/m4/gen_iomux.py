#!/usr/bin/env python3
import re
import sys

TEMPLATE="""// AUTOGENERATED, DO NOT MODIFY
#pragma once
struct pinmux_conf {{
  uint32_t mux_register;
  uint32_t config_register;
}};

const struct pinmux_conf pinmux_confs[] = {{
{pins}
}};
"""

pattern = r"IOMUXC_[^ ]+GPIO(?P<port>[0-5])_IO(?P<pin>[0-9]{2})\s+" + \
          "(?P<mux_register>0x[0-9a-fA-F]+),\s*" + \
          "(?P<mux_mode>0x[0-9a-fA-F]+),\s*" + \
          "(?P<input_register>0x[0-9a-fA-F]+),\s*" + \
          "(?P<input_daisy>0x[0-9a-fA-F]+),\s*" + \
          "(?P<config_register>0x[0-9a-fA-F]+)" 

EMPTY = {
  "mux_register": 0,
  "mux_mode": 0,
  "input_register": 0,
  "input_daisy": 0,
  "config_register": 0,
}
map = [EMPTY] * 160

for m in re.finditer(pattern, sys.stdin.read()):
  items = m.groupdict()

  offset = (int(items['port']) - 1) * 32 + int(items['pin'])
  map[offset] = items


pins = ",\n".join(["  {{{}, {}}}".format(k['mux_register'], format(k['config_register'])) for k in map])
print(TEMPLATE.format(pins=pins))
