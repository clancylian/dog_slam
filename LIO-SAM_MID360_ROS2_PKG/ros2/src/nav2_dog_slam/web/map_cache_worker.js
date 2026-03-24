// 地图缓存重建 Web Worker
// 在后台线程处理像素数据，避免阻塞主线程

// 最大缓存边长，超过此值会预先缩放
const MAX_CACHE_SIZE = 720;

self.onmessage = function(e) {
  const { mapData, width, height, colors, resolution } = e.data;
  
  const startTime = performance.now();
  
  // 计算是否需要预缩放
  const maxDimension = Math.max(width, height);
  let targetWidth = width;
  let targetHeight = height;
  let scaleFactor = 1.0;
  
  if (maxDimension > MAX_CACHE_SIZE) {
    scaleFactor = MAX_CACHE_SIZE / maxDimension;
    targetWidth = Math.round(width * scaleFactor);
    targetHeight = Math.round(height * scaleFactor);
    console.log(`[Worker] 地图预缩放: ${width}x${height} -> ${targetWidth}x${targetHeight} (scale: ${scaleFactor.toFixed(3)})`);
  }
  
  // 解析颜色
  const obstacleColor = hexToRgb(colors.obstacle);
  const possibleObstacleColor = hexToRgb(colors.possibleObstacle);
  const unknownColor = hexToRgb(colors.unknown);
  const freeColor = hexToRgb(colors.free);
  
  // 创建像素数据数组
  const totalPixels = targetWidth * targetHeight;
  const pixels = new Uint8ClampedArray(totalPixels * 4);
  
  if (scaleFactor < 1.0) {
    // 需要预缩放 - 使用最近邻插值
    for (let targetRow = 0; targetRow < targetHeight; targetRow++) {
      // 计算对应的源行（翻转Y轴）
      const srcRow = Math.round((height - 1 - (targetRow / scaleFactor)));
      const clampedSrcRow = Math.max(0, Math.min(height - 1, srcRow));
      
      for (let targetCol = 0; targetCol < targetWidth; targetCol++) {
        // 计算对应的源列
        const srcCol = Math.round(targetCol / scaleFactor);
        const clampedSrcCol = Math.max(0, Math.min(width - 1, srcCol));
        
        // 获取源数据值
        const dataIndex = clampedSrcRow * width + clampedSrcCol;
        const value = mapData[dataIndex];
        
        // 设置像素颜色
        const pixelIndex = (targetRow * targetWidth + targetCol) * 4;
        
        let color;
        if (value === -1) {
          color = unknownColor;
        } else if (value > 50) {
          color = obstacleColor;
        } else if (value > 0) {
          color = possibleObstacleColor;
        } else {
          color = freeColor;
        }
        
        pixels[pixelIndex] = color.r;
        pixels[pixelIndex + 1] = color.g;
        pixels[pixelIndex + 2] = color.b;
        pixels[pixelIndex + 3] = 255;
      }
    }
  } else {
    // 不需要预缩放 - 原始处理
    for (let row = 0; row < height; row++) {
      // 翻转行号，实现地图上下翻转
      const flippedRow = height - 1 - row;
      
      for (let col = 0; col < width; col++) {
        // 计算原始数据索引（使用翻转后的行号）
        const dataIndex = flippedRow * width + col;
        const value = mapData[dataIndex];
        
        // 计算像素索引（正常顺序）
        const pixelIndex = (row * width + col) * 4;
        
        let color;
        if (value === -1) {
          color = unknownColor;
        } else if (value > 50) {
          color = obstacleColor;
        } else if (value > 0) {
          color = possibleObstacleColor;
        } else {
          color = freeColor;
        }
        
        pixels[pixelIndex] = color.r;
        pixels[pixelIndex + 1] = color.g;
        pixels[pixelIndex + 2] = color.b;
        pixels[pixelIndex + 3] = 255;
      }
    }
  }
  
  const endTime = performance.now();
  
  // 发送结果回主线程
  self.postMessage({
    pixels: pixels,
    width: targetWidth,
    height: targetHeight,
    originalWidth: width,
    originalHeight: height,
    scaleFactor: scaleFactor,
    processTime: endTime - startTime
  }, [pixels.buffer]); // 使用 Transferable Objects 避免复制
};

// 十六进制颜色转RGB
function hexToRgb(hex) {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : { r: 0, g: 0, b: 0 };
}
