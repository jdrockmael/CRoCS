from PIL import Image
import PIL
import os

directory = "tag36h11"
new_dir = "new"
width = 1024
height = 1024

for file in os.listdir(directory):
    # filename = os.fsdecode(file)
    f = os.path.join(directory, file)

    # print(f)

    image = Image.open(f)
    image = image.resize((width, height), Image.NEAREST)
    image.save("new/" + file)
    # if filename.endswith(".asm") or filename.endswith(".py"): 
    #     # print(os.path.join(directory, filename))
    #     continue
    # else:
    #     continue



# image = Image.open('tag36_11_00031.png')
# image = image.resize((width, height), Image.NEAREST)
# image.save('resize.png')