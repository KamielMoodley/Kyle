#!/usr/bin/env python3

out = open('rx2135.xml', 'w')
out.write("""<scene>
  <description text="Starry, starry night"/>
  <integrator type="symplectic-euler" dt="0.01"/>
  <maxsimfreq max="5000"/>
""")
out.write('<duration time="10"/>\n')


nstar = 12
px = [660, 810, 522, 475, 314, 249, 221, 265, 61, 30, 81, 289,551]
py = [130, 90, 143, 44, 31, 20, 77, 195, 35, 300, 315, 342,279]
radius = [50,25,23,30,12,10,22,14,17,14,20,32,3]
for i in range(nstar):
    out.write('<particle m="'+str(radius[i]* 1000000000000)+'" px="'+str(px[i])+'" py="-'+str(py[i])+'" vx="0" vy="0" fixed="1" radius="'+str(radius[i])+'"/>\n')
out.write('<particlecolor i="1" r="'+str(220/255)+'" g="'+str(170/255)+'" b="'+str(90/255)+'"/>\n')
out.write('<particlecolor i="2" r="'+str(230/255)+'" g="'+str(165/255)+'" b="'+str(63/255)+'"/>\n')
out.write('<particlecolor i="3" r="'+str(215/255)+'" g="'+str(167/255)+'" b="'+str(55/255)+'"/>\n')
out.write('<particlecolor i="4" r="'+str(240/255)+'" g="'+str(164/255)+'" b="'+str(63/255)+'"/>\n')
out.write('<particlecolor i="5" r="'+str(232/255)+'" g="'+str(136/255)+'" b="'+str(56/255)+'"/>\n')
out.write('<particlecolor i="6" r="'+str(244/255)+'" g="'+str(169/255)+'" b="'+str(57/255)+'"/>\n')
out.write('<particlecolor i="7" r="'+str(231/255)+'" g="'+str(181/255)+'" b="'+str(96/255)+'"/>\n')
out.write('<particlecolor i="8" r="'+str(245/255)+'" g="'+str(175/255)+'" b="'+str(63/255)+'"/>\n')
out.write('<particlecolor i="9" r="'+str(210/255)+'" g="'+str(197/255)+'" b="'+str(186/255)+'"/>\n')
out.write('<particlecolor i="10" r="'+str(235/255)+'" g="'+str(202/255)+'" b="'+str(96/255)+'"/>\n')
out.write('<particlecolor i="11" r="'+str(211/255)+'" g="'+str(194/255)+'" b="'+str(154/255)+'"/>\n')

out.write('<particle m="1000000000000" px="551" py="-279" vx="0" vy="0" fixed="1" radius="0.1"/>\n')
out.write('<particlecolor i="12" r="'+str(6/255)+'" g="'+str(26/255)+'" b="'+str(65/255)+'"/>\n')
nstar += 1

count = nstar

def overlap(xx, yy):
    k = 1.1
    for i in range(nstar):
        if xx <= px[i] + radius[i]*k and xx >= px[i] - radius[i]*k \
                and yy <= py[i] + radius[i]*k and yy >= py[i] - radius[i]*k:
            return True
    return False

import random
r = [71,133,19,151,8,12,24]
g = [118,162,77,140,41,81,74]
b = [191,235,169,173,106,177,154]
for i in range(2500):
    xx = random.randint(0, 900)
    yy = random.randint(10,365)
    if overlap(xx, yy): continue
    color = random.randint(0, len(r)-1)
    out.write('<particle m="1" px="'+str(xx)+'" py="-'+str(yy)+'" vx="'+str(random.randint(-30,200))+'" vy="'+str(random.randint(-5,5))+'" fixed="0" radius="3"/>\n')
    out.write('<particlecolor i="'+str(count)+'" r="'+str(r[color]/255)+'" g="'+str(g[color]/255)+'" b="'+str(b[color]/255)+'"/>\n')
    out.write('<particlepath i="'+str(count)+'" duration="10" r="'+str(r[color]/255)+'" g="'+str(g[color]/255)+'" b="'+str(b[color]/255)+'"/>\n')
    for j in range(nstar):
        out.write('<vortexforce i="'+str(count)+'" j="'+str(j)+'" kbs="80" kvc="50000"/>\n')
    count += 1

out.write('<particle m="1" px="660" py="-130" vx="0" vy="0" fixed="1" radius="80"/>\n')
out.write('<particlecolor i="'+str(count)+'" r="'+str(250/255)+'" g="'+str(212/255)+'" b="'+str(90/255)+'"/>\n')

out.write('<backgroundcolor r="'+str(6/255)+'" g="'+str(26/255)+'" b="'+str(65/255)+'"/>\n')
out.write("""</scene>""")
out.close()
