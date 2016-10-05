#!/usr/bin/env python3

from PIL import Image
bb8img = 'clip-art-tom-and-jerry-476770.jpg'
im = Image.open(bb8img)
width, height = im.size

basewidth = 80
wpercent = (basewidth/float(width))
width = basewidth
height = int((float(height)*float(wpercent)))
im = im.resize((width, height), Image.ANTIALIAS)

# http://stackoverflow.com/questions/1109422/getting-list-of-pixel-values-from-pil
pixels = list(im.getdata())
pixels = [pixels[i * width:(i + 1) * width] for i in range(height)]


cyperc = 0.6
cxperc = 0.25
crperc = 0.25
cy = cyperc * height
cx = cxperc * width
cr = crperc * width

out = open('rx2135.xml', 'w')
out.write("""<scene>
  <description text="Run, Tom!"/>
  <integrator type="explicit-euler" dt="0.01"/>
  <maxsimfreq max="500"/>
  <halfplane px="195" py="0.0" nx="-1" ny="0"/>
  <halfplanecolor i="0" r="1" g="1" b="1"/>
  <collision type="simple"/>
  <simplegravity fx="80" fy="0"/>
  <duration time="3"/>
  <viewport cx="110" cy="-30" size="110"/>
""")
# out.write('<particle m="1" px="210" py="-'+str(height/2)+'" vx="0" vy="0" fixed="1" radius="0.01"/>\n')

def isInCircle(x, y):
    if ( (x-cx)*(x-cx) + (y-cy)*(y-cy) <= cr*cr): return True
    return False
import random

out.write('<particle m="10" px="'+str(cx)+'" py="-'+str(cy)+'" vx="0" vy="0" fixed="0" radius="0.01"/>\n')
count = 1
nedge = 0
bound = 180
for i in range(height):
    for j in range(width):
        if isInCircle(j, i):
            if random.random() < 0.7:
                continue
            rad = 0.85
            r, g, b = pixels[i][j]
            out.write('<particle m="1" px="'+str(j)+'" py="'+str(-i)+'" vx="0" vy="0" fixed="0" radius="'+str(rad)+'"/>\n')
            out.write('<particlecolor i="'+str(count)+'" r="'+str(r/255)+'" g="'+str(g/255)+'" b="'+str(b/255)+'"/>\n')
            out.write('<vortexforce i="0" j="'+str(count)+'" kbs="-80" kvc="8"/>\n')
            # out.write('<gravitationalforce i="0" j="'+str(count)+'" G="0.00001"/>\n')
        else:
            rad = 0.65
            r, g, b = pixels[i][j]
            if r >= bound and g >= bound and b >= bound: continue
            out.write('<particle m="1" px="'+str(j)+'" py="'+str(-i)+'" vx="0" vy="0" fixed="0" radius="'+str(rad)+'"/>\n')
            out.write('<particlecolor i="'+str(count)+'" r="'+str(r/255)+'" g="'+str(g/255)+'" b="'+str(b/255)+'"/>\n')
        count += 1

out.write("""
  <halfplane px="200" py="0.0" nx="-1" ny="0"/>
  <halfplanecolor i="1" r="0.1" g="0.2" b="0.3"/>
""")

# out.write('<backgroundcolor r="'+str(6/255)+'" g="'+str(26/255)+'" b="'+str(65/255)+'"/>\n')
out.write("""</scene>""")
out.close()
