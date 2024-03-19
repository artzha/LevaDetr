
# 10 classes to high contrast rgb colors in 0-255 range
LABEL_TO_RGB = [
    [255, 0, 0],    # car
    [0, 255, 0],    # truck
    [0, 0, 255],    # construction_vehicle
    [255, 255, 0],  # bus
    [0, 255, 255],  # trailer
    [255, 0, 255],  # barrier
    [128, 0, 128],# motorcycle
    [0, 128, 0],# bicycle
    [128, 128, 0],# pedestrian
    [0, 128, 128] # traffic cone
]

LABEL_REMAP = [
    0,
    0,
    0,
    0,
    0,
    1,  # barrier -> barrier
    2,  # motorcycle -> bicycle
    2,  # bicycle -> bicycle
    3,  # pedestrian -> pedestrian
    1   # traffic cone -> barrier
]

LABEL_REMAP_TO_RGB = [
    [255, 0, 0],    # car
    [128, 128, 0],  # barrier
    [0, 255, 0],    # bicycle
    [0, 0, 255],    # pedestrian
    [0, 128, 128]   # traffic cone
]