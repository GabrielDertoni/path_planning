import { RRTAlgorithm } from "../pkg";

const canvas = document.getElementById("main-canvas") as HTMLCanvasElement;
const max_iters_input = document.getElementById("max-iters-input") as HTMLInputElement;
const max_iters_display = document.getElementById("max-iters-display")!;

let rrt = new RRTAlgorithm(200000);

function main() {
    setupCanvas();
    max_iters_display.innerHTML = max_iters_input.value;

    const max_iters = parseInt(max_iters_input.value);

    while (rrt.iters() < max_iters) {
        rrt.step();
    }
    console.log("iterated ", max_iters);
    rrt.draw(canvas);

    max_iters_input.addEventListener("input", () => {
        const max_iters = parseInt(max_iters_input.value);
        let has_changed = rrt.iters() < max_iters;
        while (rrt.iters() < max_iters) {
            rrt.step();
        }
        if (has_changed) {
            rrt.draw(canvas);
        }
    });

    max_iters_input.addEventListener("change", () => {
        max_iters_display.innerHTML = max_iters_input.value;
        const max_iters = parseInt(max_iters_input.value);

        if (rrt.iters() > max_iters) {
            rrt = new RRTAlgorithm(200000);

            while (rrt.iters() < max_iters) {
                rrt.step();
            }
            rrt.draw(canvas);
        }
    })
}
main();

function setupCanvas() {
    const size = 1300;
    canvas.style.width = size + "px";
    canvas.style.height = size + "px";
    canvas.width = size;
    canvas.height = size;
}