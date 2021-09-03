import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

from utils import get_data

def viz(ground_truth):
    """
    create a grid visualization of images with color coded bboxes
    args:
    - ground_truth [list[dict]]: ground truth data
    """
    # IMPLEMENT THIS FUNCTION
    
    color_dict = {
        1 : [1, 0, 0],
        2 : [0, 1, 0]
    }

    max_row = 4
    max_col = 5

    f, ax = plt.subplots(max_row, max_col, figsize=(20, 10))

    for i in range(len(ground_truth)):
        row = i // max_col
        col = i % max_col   

        img_dict = ground_truth[i]        
        print(row, col, i, img_dict)

        filename = img_dict['filename'] 
        img = Image.open('data/images/' + filename)

        boxes = img_dict['boxes']
        classes = img_dict['classes']

 

        ax[row, col].imshow(img)

        for j in range(len(boxes)):
            y1, x1, y2, x2= boxes[j]
            box_class = classes[j]

            color = color_dict[box_class]        
            rect = Rectangle((x1, y1), x2 - x1, y2 - y1, edgecolor=color, facecolor='none')

            ax[row, col].add_patch(rect)

    plt.show()
        
        
    
    


if __name__ == "__main__": 
    ground_truth, _ = get_data()
    viz(ground_truth)