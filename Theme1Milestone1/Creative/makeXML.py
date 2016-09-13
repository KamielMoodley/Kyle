out = open('rx2135.xml', 'w')
out.write("""<scene>
  <integrator type="explicit-euler" dt="0.01"/>
  <maxsimfreq max="500.0"/>
  <simplegravity fx="0.0" fy="2"/>
""")
f = open('starwars.csv')
lines = f.readlines()
h = len(lines)
v = 50
out.write('<duration time="'+str(1.8 * h / v)+'"/>\n')
# for centerCamera(); should set in main.cpp the number of centering particles as 4
out.write('<particle m="1.0" px="0" py="'+str(h / 2)+'" vx="0" vy="0" fixed="1" radius="0.1"/>\n')
out.write('<particle m="1.0" px="0" py="'+str(- h / 2)+'" vx="0" vy="0" fixed="1" radius="0.1"/>\n')
out.write('<particle m="1.0" px="'+str(h / 8)+'" py="0" vx="0" vy="0" fixed="1" radius="0.1"/>\n')
out.write('<particle m="1.0" px="'+str(- h / 8)+'" py="0" vx="0" vy="0" fixed="1" radius="0.1"/>\n')

i = - h / 2
count = 0
for line in lines:
    bits = line[:-1].split(',')
    j = - len(bits) / 2 - 1
    for bit in bits:
        j += 1
        if bit == "1": continue
        t = (h / 2 - i * 3) / v
        vx = - j / t
        out.write('<particle m="1.0" px="'+str(j)+'" py="'+str(i)+'" vx="'+str(vx)+'" vy="'+str(v)+'" fixed="0" radius="0.9"/>\n')
        # http://paletton.com/#uid=11b0u0kyYqtn3ylu4wrDjlAKEgh
        # shadow
        out.write('<particlepath i="'+str(count)+'" duration="'+str(t/500)+'" r="0.67" g="0.506" b="0"/>\n')
        out.write('<particlecolor i="'+str(count)+'" r="1" g="0.82" b="0.278"/>')
        count += 1
    i -= 1
f.close()

out.write("""  <backgroundcolor r="0.0" g="0.0" b="0.0"/>
</scene>
""")
out.close()
