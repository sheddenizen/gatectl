#!/usr/bin/python3

data = [ None ] * 4096

with open('cal2-data.csv') as f:
  lines = f.readlines()

for line in lines:
  values = line.split(',')
  if len(values) == 5:
    #print(f'Add to entry {int(values[3])}, {int(values[4])}')
#    idx = int(values[3])
#    val=int(values[4])
    idx = (int(values[1]) + int(values[3])) // 2
    val=int(values[4])
    if data[idx] is None:
      data[idx] = [ val ]
    else:
      data[idx].append(val)
#  else:
#    print(f'Rejected line "{line}"')

avgs = [ None ] * 4096

for i,v in enumerate(data):
  if v is None:
#    print(f'{i},,,,')
    continue
  # print((x,y))
  vmod = [ x % 12800 for x in v ]
  if max(vmod) - min(vmod) > 6400:
    vmodo = [ (x + 6400) % 12800 for x in v ]
    avg = (sum(vmodo)/len(vmodo) + 6400) % 12800
    spread = max(vmodo) - min(vmodo)
  else:
    avg = (sum(vmod)/len(vmod) % 12800)
    spread = max(vmod) - min(vmod)

#  print (f'{i},{round(avg)},{spread},{len(v)},{",".join([str(v) for v in vmod])}')
  avgs[i] = avg


W=127
smoo = []
for i in range(len(avgs)):
  total = 0
  count = 0
  v = avgs[(i-W//2+1)%4096] if avgs[(i-W//2) % 4096] is None else avgs[(i-W//2) % 4096]
  offs = 6400 if v > 9600 else 0
  for j in range(-W//2, W//2+1):
    v = avgs[(i+j)%4096]
    if v is None:
      continue
    total += (v + offs) % 12800
    count += 1
  smoo.append(round(total/count - offs) % 12800)

lu=[]

for x,y in enumerate(smoo):
  a=avgs[x]
  n=(x*12800)//4096
  print(f'{x} {a} {y} {round(y-a) % 12800 if a is not None else 0} {n} { (y-n) % 12800 }')
  lu.append((y-n) % 12800)

#EPL=32
#for c in range(0,4096, EPL):
#  print(', '.join([ str(e) for e in lu[c:c+EPL]])+',')
