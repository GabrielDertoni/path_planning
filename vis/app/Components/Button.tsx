import { MouseEventHandler, ReactNode } from "react";
import "./Button.scss";

interface Props {
    onClick?: MouseEventHandler,
    children?: ReactNode,
}

export const Button = ({ children, onClick }: Props) => {
    return (
        <div className="button-wrapper">
            <button onClick={onClick} className="menu-button">
                { children }
            </button>
        </div>
    );
}