import { RefObject, createRef, useEffect } from "react";

import { Button } from "./Button";
import { Algorithm } from "./Algorithms";
import "./Menu.scss";

interface Props {
    onClick: (option: Algorithm) => void;
}

export const Menu = ({ onClick }: Props) => {
    const background: RefObject<HTMLCanvasElement> = createRef();

    useEffect(() => {
        let canvas = background.current!;
        let width = canvas.offsetWidth;
        let height = canvas.offsetHeight;
        canvas.width = width;
        canvas.height = height;

        let ctx = canvas.getContext("2d")!;

        ctx.translate(width / 2, height / 2);

        let phi = randomInRange(0, 2 * Math.PI);
        let numShapes = randomInRange(4, 10);

        for (let i = 0; i < numShapes; i++) {
            let ang = phi + (i * 2 * Math.PI) / numShapes;
            let dist = 500;
            drawRandomRect(
                ctx,
                dist * Math.cos(ang),
                dist * Math.sin(ang),
                randomInRange(100, 300),
                randomInRange(100, 300)
            );
        }
    }, []);

    return (
        <div className="menu">
            <div className="background">
                <div className="background mask"></div>
                <canvas className="background" ref={background}></canvas>
            </div>

            <h1 className="title">RRT Algorithms</h1>
            <Button onClick={() => onClick(Algorithm.RRT)}>RRT</Button>
            <Button onClick={() => onClick(Algorithm.RRTStar)}>RRT*</Button>
            <Button onClick={() => onClick(Algorithm.RRTConnect)}>RRT-Connect</Button>
            <Button onClick={() => onClick(Algorithm.RRTStarSmart)}>RRT*-Smart</Button>
        </div>
    );
};

function drawRandomRect(
    ctx: CanvasRenderingContext2D,
    centerX: number,
    centerY: number,
    width: number,
    height: number
) {
    const colors = ["#D9AE15", "#E10000", "#EB002A", "#EB5500"];

    const prevTransform = ctx.getTransform();

    ctx.fillStyle = chooseRandom(colors);
    ctx.translate(centerX, centerY);
    ctx.rotate(Math.random() * Math.PI);
    ctx.fillRect(-width / 2, -height / 2, width, height);

    ctx.setTransform(prevTransform);
}

function chooseRandom<T>(options: Array<T>): T {
    const idx = Math.floor(Math.random() * options.length);
    return options[idx];
}

function randomInRange(min: number, max: number): number {
    return min + Math.random() * (max - min);
}
