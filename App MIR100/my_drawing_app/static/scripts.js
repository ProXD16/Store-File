let canvas;
let ctx;
let mapInfo;
let drawingTool = 'line'; // Mặc định là vẽ đường thẳng
let isDrawing = false;
let startX, startY;
let points = [];

async function loadMapInfo() {
  try {
    const response = await fetch('/static/map_image.info'); // Thay đổi đường dẫn nếu cần
    const text = await response.text();
    mapInfo = {};
    text.split('\n').forEach(line => {
      const [key, value] = line.split(': ').map(s => s.trim());
      if (key && value) {
        mapInfo[key] = parseFloat(value); // Chuyển đổi giá trị thành số
      }
    });
    console.log("Map Info:", mapInfo);
  } catch (error) {
    console.error("Error loading map info:", error);
  }
}

function init() {
  canvas = document.getElementById('drawingCanvas');
  ctx = canvas.getContext('2d');

  loadMapInfo().then(() => {
    const img = new Image();
    img.onload = function() {
      canvas.width = img.width;
      canvas.height = img.height;
      ctx.drawImage(img, 0, 0);
    };
    img.src = '/static/map_image.png'; // Thay đổi đường dẫn nếu cần
  });

  canvas.addEventListener('mousedown', startDrawing);
  canvas.addEventListener('mouseup', stopDrawing);
  canvas.addEventListener('mouseout', stopDrawing);
  canvas.addEventListener('mousemove', draw);

  // Setup các nút
  document.getElementById('drawLine').addEventListener('click', () => setDrawingTool('line'));
  document.getElementById('drawArc').addEventListener('click', () => setDrawingTool('arc'));
  document.getElementById('drawPolyline').addEventListener('click', () => setDrawingTool('polyline'));
  document.getElementById('drawSpline').addEventListener('click', () => setDrawingTool('spline'));
  document.getElementById('clearCanvas').addEventListener('click', clearCanvas);
  document.getElementById('saveLines').addEventListener('click', saveLines);
}

function setDrawingTool(tool) {
  drawingTool = tool;
  points = []; // Reset points khi đổi tool, đặc biệt quan trọng với polyline
  console.log("Tool changed to:", drawingTool);
}


function startDrawing(e) {
  isDrawing = true;
  startX = e.offsetX;
  startY = e.offsetY;
  points.push({x: startX, y: startY});
}

function stopDrawing() {
  isDrawing = false;
  if (drawingTool === 'polyline') {
    // Khi kết thúc polyline, không reset canvas ngay.
  } else {
    points = []; // Reset points cho các tool khác
  }
}


function draw(e) {
  if (!isDrawing) return;
  const x = e.offsetX;
  const y = e.offsetY;

  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.strokeStyle = 'red';

  switch (drawingTool) {
    case 'line':
      drawLine(startX, startY, x, y);
      break;
    case 'arc':
      drawArc(startX, startY, x, y);
      break;
    case 'polyline':
      drawPolyline(x, y);
      break;
    case 'spline':
      drawSpline(); // Cần nhiều hơn 2 điểm để vẽ spline
      break;
  }
}

function drawLine(x1, y1, x2, y2) {
  clearCanvas(); // Xóa canvas trước khi vẽ lại đường thẳng mới
  drawImage(); // Vẽ lại hình ảnh bản đồ
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.closePath();
}

function drawArc(x1, y1, x2, y2) {
  clearCanvas();
  drawImage();
  const radius = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  ctx.beginPath();
  ctx.arc(x1, y1, radius, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.closePath();
}

function drawPolyline(x, y) {
  if (!isDrawing) return;

  points.push({x: x, y: y});

  clearCanvas();
  drawImage();

  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.stroke();
  ctx.closePath();
}

function drawSpline() {
    if (points.length < 3) return; // Cần ít nhất 3 điểm

    clearCanvas();
    drawImage();

    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);

    for (let i = 0; i < points.length - 1; i++) {
        let p0 = (i > 0) ? points[i - 1] : points[i];
        let p1 = points[i];
        let p2 = points[i + 1];
        let p3 = (i < points.length - 2) ? points[i + 2] : points[i + 1];

        // Tính toán điểm điều khiển
        let cp1x = p1.x + (p2.x - p0.x) / 6;
        let cp1y = p1.y + (p2.y - p0.y) / 6;

        let cp2x = p2.x - (p3.x - p1.x) / 6;
        let cp2y = p2.y - (p3.y - p1.y) / 6;

        ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, p2.x, p2.y);
    }

    ctx.stroke();
    ctx.closePath();
}


function clearCanvas() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
}

function drawImage() {
  const img = new Image();
  img.onload = function() {
    ctx.drawImage(img, 0, 0);
  }
  img.src = '/static/map_image.png';
}

function saveLines() {
    //TODO: Implement save lines functionality
    alert("Save lines functionality not implemented yet!");
}

window.onload = init;