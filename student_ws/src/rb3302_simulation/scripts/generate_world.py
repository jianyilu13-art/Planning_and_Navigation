#!/usr/bin/env python3
"""Convert a P5 map into merged 0.05 m wall runs in an SDF fragment.
Unknown (205) and occupied cells are walls; free cells are 254/255. Review the
generated fragment visually before using it for collision validation.
"""
import argparse
from pathlib import Path
def main():
 p=argparse.ArgumentParser(); p.add_argument('pgm'); p.add_argument('output'); p.add_argument('--resolution',type=float,default=.05); p.add_argument('--origin',nargs=2,type=float,default=[-.892,-3.707]); a=p.parse_args()
 raw=Path(a.pgm).read_bytes().split(b'\n',3); w,h=map(int,raw[1].split()); data=raw[3]; models=[]; n=0
 for r in range(h):
  c=0
  while c<w:
   # Only map values classified free by trinary maps (254/255) are traversable.
   # Occupied and unknown (205) cells remain collision geometry.
   if data[r*w+c] >= 254: c+=1; continue
   start=c
   while c<w and data[r*w+c] < 254: c+=1
   length=c-start; x=a.origin[0]+(start+length/2)*a.resolution; y=a.origin[1]+(h-r-.5)*a.resolution
   models.append(f'<model name="wall_{n}"><static>true</static><pose>{x} {y} .4 0 0 0</pose><link name="l"><collision name="c"><geometry><box><size>{length*a.resolution} {a.resolution} .8</size></box></geometry></collision><visual name="v"><geometry><box><size>{length*a.resolution} {a.resolution} .8</size></box></geometry></visual></link></model>'); n+=1
 # A self-contained Gazebo model is loadable through model://course_walls.
 output = '<sdf version="1.9"><model name="course_walls"><static>true</static>' + '\n'.join(models) + '</model></sdf>'
 Path(a.output).write_text(output); print(f'wrote {n} collision runs')
if __name__ == '__main__': main()
