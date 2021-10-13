import { useState, createRef, useEffect, RefObject } from "react";
import { RRTAlgorithm, init } from "../pkg";

let rrt = new RRTAlgorithm();

export const App = () => {
    let width = 1300;
    let height = 1300;
    const iters_min = 200;
    const iters_max = 200000;

    const [iters, setIters] = useState(200);
    const iters_input: RefObject<HTMLInputElement> = createRef();
    const main_canvas: RefObject<HTMLCanvasElement> = createRef();

    useEffect(() => {
        init();

        while (rrt.iters() < iters_min) {
            rrt.step();
        }
        rrt.draw(main_canvas.current!);
    }, []);

    function onInputIters(e: any) {
        const max_iters = parseInt(iters_input.current!.value);
        setIters(max_iters);
        const has_changed = rrt.iters() < max_iters;
        while (rrt.iters() < max_iters) {
            rrt.step();
        }
        if (has_changed) {
            rrt.draw(main_canvas.current!);
        }
    }

    function onChangeIters(e: any) {
        const max_iters = parseInt(iters_input.current!.value);

        if (rrt.iters() > max_iters) {
            rrt = new RRTAlgorithm();

            while (rrt.iters() < max_iters) {
                rrt.step();
            }
            rrt.draw(main_canvas.current!);
        }
    }

    return (
        <div>
            <input
                className="wide-slider"
                ref={iters_input}
                type="range"
                min={iters_min}
                max={iters_max}
                onInput={onInputIters}
                onChange={onChangeIters}
            />
            <p className="iter-count">
                Iterations: <span> {iters} </span>
            </p>
            <canvas ref={main_canvas} width={width} height={height}></canvas>
        </div>
    );
}