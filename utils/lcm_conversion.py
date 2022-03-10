
dir = "C:\\Users\\borin\\Documents\\GitHub\\Rover-Simulator\\misc\\"
map_name = "CMap.asc"
map = open(dir + map_name, "r")

contents  = map.read()
array = contents.split()
map.close()

new_dir = "C:\\Users\\borin\\Documents\\GitHub\\Rover-Simulator\\maps\\"
the_map =   "TL16NW"
new_map_name = the_map + "_landcover.asc"
new_map = open(new_dir + new_map_name, "w")

new_array = array[2::3]
if(len(new_array)  == 1000000):
    holding_array = []
    for i in range(0, 1000000, 1000):
        x = ' '.join(new_array[i:i+1000])
        x = x + '\n'
        holding_array.append(x)

e_map_name = the_map + "_elevation.asc"
e_map = open(new_dir + e_map_name, "r")
elevation_content = e_map.read()
e_map.close()
count = 0
index = -1
for i in range(len(elevation_content)):
    if(elevation_content[i] == '\n'):
        count +=1
    if(count == 5):
        index = i
        break

metadata = elevation_content[:index+1]

new_map.write(metadata)

for i in range(len(holding_array)):
    new_map.write(holding_array[i])

new_map.close()
print("Done")