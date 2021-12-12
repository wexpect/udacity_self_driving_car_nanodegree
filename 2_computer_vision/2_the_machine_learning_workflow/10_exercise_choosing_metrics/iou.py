import numpy as np

from utils import get_data, check_results


def calculate_ious(gt_bboxes, pred_bboxes):
    """
    calculate ious between 2 sets of bboxes 
    args:
    - gt_bboxes [array]: Nx4 ground truth array
    - pred_bboxes [array]: Mx4 pred array
    returns:
    - iou [array]: NxM array of ious
    """
    ious = np.zeros((gt_bboxes.shape[0], pred_bboxes.shape[0]))
    for i, gt_bbox in enumerate(gt_bboxes):
        for j, pred_bbox in enumerate(pred_bboxes):
            ious[i,j] = calculate_iou(gt_bbox, pred_bbox)
    return ious


def calculate_iou(gt_bbox, pred_bbox):
    """
    calculate iou 
    args:
    - gt_bbox [array]: 1x4 single gt bbox
    - pred_bbox [array]: 1x4 single pred bbox
    returns:
    - iou [float]: iou between 2 bboxes
    """
    ## IMPLEMENT THIS FUNCTION
    
    g = gt_bbox
    p = pred_bbox
    
    max_x = np.max([g[2], p[2]])
    max_x
    
    max_y = np.max([g[3], p[3]])
    max_y
    
    mat_g = np.zeros((max_y, max_x))
    mat_g.shape
    
    mat_g[g[1]:g[3], g[0]:g[2]] = 1
    mat_g
    
    mat_p = np.zeros((max_y, max_x))
    mat_p.shape
    
    mat_p[p[1]:p[3], p[0]:p[2]] = 1
    mat_p

    count_one_mat_g = np.sum(mat_g == 1)
    count_one_mat_g
    
    count_one_mat_p = np.sum(mat_p == 1)
    count_one_mat_p
    
    mat_g_intersect_mat_p = np.sum(mat_g + mat_p == 2)
    mat_g_intersect_mat_p    
    
    mat_g_union_mat_p = count_one_mat_g + count_one_mat_p - mat_g_intersect_mat_p
    mat_g_union_mat_p    
    
    iou = mat_g_intersect_mat_p / mat_g_union_mat_p
    iou

    return iou


if __name__ == "__main__": 
    ground_truth, predictions = get_data()
    
    print('ground_truth', ground_truth)
    print('predictions', predictions)
    
    # get bboxes array
    filename = 'segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png'
    gt_bboxes = [g['boxes'] for g in ground_truth if g['filename'] == filename][0]
    print('gt_bboxes', gt_bboxes)
        
    gt_bboxes = np.array(gt_bboxes)
    pred_bboxes = [p['boxes'] for p in predictions if p['filename'] == filename][0]
    pred_boxes = np.array(pred_bboxes)
    print('pred_bboxes', pred_bboxes)    
    
    ious = calculate_ious(gt_bboxes, pred_boxes)
    check_results(ious)