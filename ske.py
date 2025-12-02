# skeleton_extractor.py
import cv2
import numpy as np
from skimage.morphology import skeletonize
from skimage import io
import matplotlib.pyplot as plt

def extract_skeleton_from_image(image_path, output_txt='trajectory.txt'):
    """
    从图像提取骨架并保存为轨迹点
    
    参数:
        image_path: 输入图像路径
        output_txt: 输出轨迹点文件路径
    """

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"无法读取图像: {image_path}")
        return None
    
    # 二值化
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    
    # 处理去噪
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    
    #  骨架化
    binary_bool = binary > 0
    skeleton = skeletonize(binary_bool)
    
    #提取骨架点坐标
    points = np.argwhere(skeleton > 0)
    sample_step=6
    # 均匀抽取：每隔sample_step个点取一个
    points = points[::sample_step]  # 关键修改！
    if len(points) == 0:
        print("未检测到骨架点！")
        return None
    
    #按照路径顺序排序（从左到右，从上到下）
    points = points[points[:, 0].argsort()]
    
    #归一化到 [0, 1] 范围
    h, w = img.shape
    normalized_points = points.astype(float)
    normalized_points[:, 0] /= h  # y坐标
    normalized_points[:, 1] /= w  # x坐标
    
    #保存为文本文件（每行：x y）
    with open(output_txt, 'w') as f:
        f.write(f"{len(normalized_points)}\n")  
        for point in normalized_points:
            y, x = point  
            f.write(f"{x:.6f} {y:.6f}\n")
    
    print(f"成功提取 {len(normalized_points)} 个骨架点，保存到 {output_txt}")
    
    # 可视化
    plt.figure(figsize=(12, 4))
    
    plt.subplot(131)
    plt.imshow(img, cmap='gray')
    plt.title('Original')
    plt.axis('off')
    
    plt.subplot(132)
    plt.imshow(binary, cmap='gray')
    plt.title('Binary')
    plt.axis('off')
    
    plt.subplot(133)
    plt.imshow(skeleton, cmap='gray')
    plt.title('Skeleton')
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig('skeleton_visualization.png')
    plt.show()
    
    return normalized_points




# MNIST示例
def extract_from_mnist_digit(digit_index=0, output_txt='mnist_trajectory.txt'):
    """
    从MNIST数据集提取特定数字的骨架
    """
    from tensorflow import keras
    
    # 加载MNIST
    (x_train, y_train), _ = keras.datasets.mnist.load_data()
    

    idx = np.where(y_train == digit_index)[0][0]
    img = x_train[idx]
    
    # 保存临时图像
    cv2.imwrite('temp_mnist.png', img)
    
    # 提取骨架
    points = extract_skeleton_from_image('temp_mnist.png', output_txt)
    
    return points


if __name__ == "__main__":

    extract_skeleton_from_image('your_image.png', 'trajectory.txt')
    

    extract_from_mnist_digit(digit_index=5, output_txt='digit5_trajectory.txt')
    
