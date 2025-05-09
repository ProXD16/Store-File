// assets/draw_arc.js
document.addEventListener('DOMContentLoaded', function () {
    const graph = document.getElementById('graph'); // Lấy phần tử biểu đồ
    const canvas = document.createElement('canvas'); // Tạo canvas để vẽ
    canvas.width = graph.clientWidth;
    canvas.height = graph.clientHeight;
    canvas.style.position = 'absolute';
    canvas.style.top = '0';
    canvas.style.left = '0';
    canvas.style.pointerEvents = 'none'; // Đảm bảo canvas không chặn sự kiện chuột
    graph.appendChild(canvas);

    const ctx = canvas.getContext('2d');
    let isDrawing = false;
    let startPoint = null;

    // Bắt đầu vẽ
    graph.addEventListener('mousedown', function (e) {
        isDrawing = true;
        startPoint = { x: e.offsetX, y: e.offsetY };
    });

    // Kết thúc vẽ
    graph.addEventListener('mouseup', function (e) {
        if (isDrawing && startPoint) {
            const endPoint = { x: e.offsetX, y: e.offsetY };
            drawArc(startPoint, endPoint);
            isDrawing = false;
            startPoint = null;
        }
    });

    // Vẽ cung tròn
    function drawArc(start, end) {
        const center = {
            x: (start.x + end.x) / 2,
            y: (start.y + end.y) / 2,
        };
        const radius = Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2)) / 2;
        const startAngle = Math.atan2(start.y - center.y, start.x - center.x);
        const endAngle = Math.atan2(end.y - center.y, end.x - center.x);

        ctx.beginPath();
        ctx.arc(center.x, center.y, radius, startAngle, endAngle);
        ctx.strokeStyle = 'red';
        ctx.lineWidth = 2;
        ctx.stroke();
    }
});