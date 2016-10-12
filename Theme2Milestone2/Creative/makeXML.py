#!/usr/bin/env python3

from PIL import Image
bb8img = 'bubble-07.jpg'
im = Image.open(bb8img)
width, height = im.size

basewidth = 40
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
  <description text=""/>
  <duration time="2.0"/>
  <integrator type="explicit-euler" dt="0.01"/>
  <collision type="continuous-time"/>
  <viewport cx="20" cy="-20" size="14"/>
  <maxsimfreq max="500"/>
""")
# out.write('<particle m="1" px="210" py="-'+str(height/2)+'" vx="0" vy="0" fixed="1" radius="0.01"/>\n')

def isInCircle(x, y):
    if ( (x-cx)*(x-cx) + (y-cy)*(y-cy) <= cr*cr): return True
    return False
import random

count = 0
bound = 240
for i in range(8,height-8):
    for j in range(8,width-8):
        xx = random.randint(0, 900)
        yy = random.randint(10,365)
        r, g, b = pixels[i][j]
        if r >= bound and g >= bound and b >= bound: continue
        out.write('<particle m="1" px="'+str(j)+'" py="-'+str(i)+'" vx="'+str(random.randint(-15,15))+'" vy="'+str(random.randint(-50,50))+'" fixed="0" radius="0.8"/>\n')
        out.write('<particlecolor i="'+str(count)+'" r="'+str(r/255)+'" g="'+str(g/255)+'" b="'+str(b/255)+'"/>\n')
        count += 1

out.write('<backgroundcolor r="'+str(6/255)+'" g="'+str(26/255)+'" b="'+str(65/255)+'"/>\n')
out.write("""</scene>""")
out.close()
