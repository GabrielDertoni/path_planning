import { Button } from "./Button";
import { Algorithm } from "./Algorithms";
import "./Menu.scss";

interface Props {
    onClick: (option: Algorithm) => void,
}

export const Menu = ({ onClick }: Props) => {
    return (
        <div className="menu">
            <h1 className="title">RRT Algorithms</h1>
            <Button onClick={() => onClick(Algorithm.RRT)}>RRT</Button>
            <Button onClick={() => onClick(Algorithm.RRTStar)}>RRT*</Button>
            <Button onClick={() => onClick(Algorithm.RRTConnect)}>RRT-Connect</Button>
            <Button onClick={() => onClick(Algorithm.RRTStarSmart)}>RRT*-Smart</Button>
        </div>
    );
}