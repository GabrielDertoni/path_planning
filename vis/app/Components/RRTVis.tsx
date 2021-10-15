import { RefObject, createRef, useState, useEffect } from "react";

import { init, RRTAlgorithm } from "../../pkg";
import { Algorithm } from "./Algorithms";
import "./RRTVis.scss";

interface Props {
    algorithm: Algorithm;
}

let rrt = new RRTAlgorithm();
let global_iters = { val: 200 };
let is_loop_running = false;

export const RRTVis = ({ algorithm }: Props) => {
    const iters_min = 200;
    const iters_max = 200000;

    const [iters, setIters] = useState(iters_min);
    const [rrt_iters, setRRTIters] = useState(iters_min);
    const main_canvas: RefObject<HTMLCanvasElement> = createRef();
    const parent_ref: RefObject<HTMLDivElement> = createRef();

    function requestAnimationFrameIterateUntil(canvas: HTMLCanvasElement) {
        let n_per_frame = 5000;
        let has_updated = rrt.iters() < global_iters.val;
        for (let i = 0; rrt.iters() < global_iters.val && i < n_per_frame; i++) {
            rrt.step();
        }

        if (rrt.iters() < global_iters.val) {
            is_loop_running = true;
            requestAnimationFrame(() => requestAnimationFrameIterateUntil(canvas));
        } else {
            is_loop_running = false;
        }

        if (has_updated) {
            rrt.draw(canvas);
            setRRTIters(rrt.iters());
        }
    }

    useEffect(() => {
        init();

        let canvas = main_canvas.current!;
        let ctx = canvas.getContext("2d")!;
        ctx.canvas.width = window.innerWidth;
        ctx.canvas.height = window.innerHeight;

        while (rrt.iters() < iters) {
            rrt.step();
        }
        rrt.draw(canvas);
    }, []);

    return (
        <div ref={parent_ref} className="fit-parent">
            <div>
                <input
                    className="wide-slider"
                    type="range"
                    min={iters_min}
                    max={iters_max}
                    onMouseUp={e => {
                        if (rrt.iters() > iters) {
                            rrt = new RRTAlgorithm();
                            if (!is_loop_running) {
                                requestAnimationFrameIterateUntil(main_canvas.current!);
                            }
                        }
                    }}
                    onChange={e => {
                        setIters(parseInt(e.target.value));
                        global_iters.val = iters;
                        if (!is_loop_running) {
                            requestAnimationFrameIterateUntil(main_canvas.current!);
                        }
                    }}
                />
                <input
                    className="wide-slider"
                    type="range"
                    min={iters_min}
                    max={iters_max}
                    value={rrt_iters}
                />
                <p className="iter-count text-black">
                    Iterations: <span> {iters} </span>
                </p>
            </div>
            <canvas className="fit-parent main-canvas" ref={main_canvas}></canvas>
        </div>
    );
};
