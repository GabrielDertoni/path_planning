import { useState } from "react";

import { Menu } from "./Menu";
import { RRTVis } from "./RRTVis";
import { Algorithm } from "./Algorithms";
import "./App.scss";
import "./Global.scss";

export const App = () => {
    let [algorithm, setAlgorithm] = useState<Algorithm | null>(null);

    function route(algorithm: Algorithm | null) {
        if (algorithm === null) {
            return <Menu onClick={algo => {
                setAlgorithm(algo);
                console.log(algo);
            }}/>;
        } else {
            return <RRTVis algorithm={algorithm}/>;
        }
    }

    return route(algorithm);
}