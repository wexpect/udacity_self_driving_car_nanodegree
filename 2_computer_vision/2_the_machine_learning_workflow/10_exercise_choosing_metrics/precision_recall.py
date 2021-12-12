import numpy as np

from iou import calculate_ious
from utils import get_data


def precision_recall(ious, gt_classes, pred_classes):
    """
    calculate precision and recall
    args:
    - ious [array]: NxM array of ious
    - gt_classes [array]: 1xN array of ground truth classes
    - pred_classes [array]: 1xM array of pred classes
    returns:
    - precision [float]
    - recall [float]
    """

    # IMPLEMENT THIS FUNCTION
    tp = 0
    fp = 0
    for i in range(len(gt_classes)):
        for j in range(len(pred_classes)):
            if ious[i][j] > 0.5:
                if gt_classes[i] == pred_classes[j]:
                    tp+=1
                else:
                    fp+=1    

    tp, fp
    
    fn = np.sum(np.max(ious, axis=1) <= 0.5)
    fn
    
    precision = tp / (tp + fp)
    precision    
    
    recall = tp / (tp + fn)
    recall    
        
    return precision, recall


if __name__ == "__main__": 
    ground_truth, predictions = get_data()
    
    # get bboxes array
    filename = 'segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png'
    gt_bboxes = [g['boxes'] for g in ground_truth if g['filename'] == filename][0]
    gt_bboxes = np.array(gt_bboxes)
    gt_classes = [g['classes'] for g in ground_truth if g['filename'] == filename][0]

    pred_bboxes = [p['boxes'] for p in predictions if p['filename'] == filename][0]
    pred_boxes = np.array(pred_bboxes)
    pred_classes = [p['classes'] for p in predictions if p['filename'] == filename][0]

    print('gt_bboxes', gt_bboxes)        
    print('gt_classes', gt_classes)  
    print('pred_boxes', pred_boxes)  
    print('pred_classes', pred_classes)
    
    ious = calculate_ious(gt_bboxes, pred_boxes)
    print('ious\n', ious)    
    
    precision, recall = precision_recall(ious, gt_classes, pred_classes)
    print('solution', precision, recall)