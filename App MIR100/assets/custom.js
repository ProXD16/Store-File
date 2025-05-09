window.dash_clientside = Object.assign({}, window.dash_clientside, {
    clientside: {
        addCustomDrawMode: function() {
            if (!Plotly || !Plotly.d3) {
                console.error("Plotly chưa được tải!");
                return;
            }

            // Thêm chế độ 'drawarc' vào modeBar
            Plotly.d3.selectAll(".modebar-btn").each(function() {
                if (this.getAttribute("data-title") === "drawarc") {
                    return;  // Đã tồn tại, không cần thêm lại
                }
            });

            // Gán sự kiện khi chọn chế độ drawarc
            var modebar = document.querySelector(".modebar-container");
            if (modebar) {
                var btn = document.createElement("button");
                btn.className = "modebar-btn";
                btn.setAttribute("data-title", "drawarc");
                btn.innerHTML = "🎯"; // Biểu tượng tùy chỉnh
                btn.onclick = function() {
                    Plotly.relayout("graph", { dragmode: "drawarc" });
                };
                modebar.appendChild(btn);
            }
        }
    }
});

// Chạy khi trang tải xong
document.addEventListener("DOMContentLoaded", function() {
    window.dash_clientside.clientside.addCustomDrawMode();
});
