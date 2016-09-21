#!/usr/bin/env python3

from PIL import Image
bb8img = '364693_1.jpg'
im = Image.open(bb8img)
width, height = im.size

basewidth = 300
wpercent = (basewidth/float(width))
width = basewidth
height = int((float(height)*float(wpercent)))
im = im.resize((width, height), Image.ANTIALIAS)

# http://stackoverflow.com/questions/1109422/getting-list-of-pixel-values-from-pil
pixels = list(im.getdata())
pixels = [pixels[i * width:(i + 1) * width] for i in range(height)]

out = open('rx2135.xml', 'w')
out.write("""<scene>
  <description text="BB-8, A Rolling Robot in a Galaxy Far, Far Away."/>
  <integrator type="symplectic-euler" dt="0.05"/>
  <maxsimfreq max="500"/>
""") # dt = 0.01
out.write('<duration time="20"/>\n')

diameter = 230 - 69
center = ( (69+230)/2, 271-diameter/2)
headY = center[1] - diameter/2 + 6
omega = 1
out.write('<particle m="1" px="'+str(center[0])+'" py="'+str(-center[1])+'" vx="0" vy="0" fixed="1" radius="0"/>\n')

from math import sqrt
def getV(x, y):
    vx = center[0] - x
    vy = center[1] - y
    rsquare = vx*vx + vy*vy
    vx *= omega
    vy *= omega
    return (vy, -vx, sqrt(rsquare))

count = 1
nedge = 0
bound = 200
for i in range(height):
    for j in range(width):
        r, g, b = pixels[i][j]
        if r >= bound and g >= bound and b >= bound: continue
        # if j < left: left = j
        # if j > right: right = j
        if (i > headY):
            # (vy, vx, R) = getV(j, i)
            (vx, vy, R) = getV(j, i)
            out.write('<particle m="1" px="'+str(j)+'" py="'+str(-i)+'" vx="'+str(vx)+'" vy="'+str(-vy)+'" fixed="0" radius="0.8"/>\n')
            out.write('<gravitationalforce i="0" j="'+str(count)+'" G="'+str(omega * omega * (R**3))+'"/>\n')
            count += 1
        else:
            out.write('<particle m="1" px="'+str(j)+'" py="'+str(-i)+'" vx="0" vy="0" fixed="0" radius="0.9"/>\n')
            out.write('<particle m="1" px="'+str(j)+'" py="'+str(-i-80)+'" vx="0" vy="0" fixed="0" radius="0"/>\n')
            out.write('<edge i="'+str(count)+'" j="'+str(count+1)+'"/>')
            out.write('<edgecolor i="'+str(nedge)+'" r="1" g="1" b="1"/>')
            out.write('<springforce edge="'+str(nedge)+'" k="10" l0="100" />')
            count += 2
            nedge += 1
        out.write('<particlecolor i="'+str(count)+'" r="'+str(r/255)+'" g="'+str(g/255)+'" b="'+str(b/255)+'"/>\n')
for i in range(int(center[1] + diameter/2 - 6), height):
    for j in range(width):
        r, g, b = pixels[i][j]
        if r >= bound and g >= bound and b >= bound: continue
        ii = center[1]-(i-center[1]) + 1
        # (vy, vx, R) = getV(j, ii)
        (vx, vy, R) = getV(j, ii)
        out.write('<particle m="1" px="'+str(j)+'" py="'+str(-ii)+'" vx="'+str(vx)+'" vy="'+str(-vy)+'" fixed="0" radius="0.8"/>\n')
        out.write('<gravitationalforce i="0" j="'+str(count)+'" G="'+str(omega * omega * (R**3))+'"/>\n')
        out.write('<particlecolor i="'+str(count)+'" r="'+str(r/255)+'" g="'+str(g/255)+'" b="'+str(b/255)+'"/>\n')
        count += 1


out.write("""  <backgroundcolor r="1" g="1" b="1"/>
</scene>
""")
out.close()
