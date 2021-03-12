import React, { useContext } from 'react';
import { lineColors, lineSizes } from '../../Lines/Line';
import { lineSelectKeys } from '../../Buttons';
import { RootContext } from '../../../RootContext';
import { useInteractionEvent } from '../../../../Common/hooks';
import { fieldSides } from '../NonInteractive/Field';

type NumberFieldProps = {
    value: number | string | undefined,
    nullValue: string,
    min: number,
    max: number,
    color: lineColors,
    side?: fieldSides,
    size: lineSizes,
    selectedCallback: (value: number) => any,
    lsk: lineSelectKeys
}
export const NumberInputField: React.FC<NumberFieldProps> = (
    {
        value,
        nullValue,
        min,
        max,
        color,
        side,
        size,
        selectedCallback,
        lsk,
    },
) => {
    const [scratchpad, setScratchpad, , ] = useContext(RootContext); // eslint-disable-line array-bracket-spacing
    let numVal;
    if (typeof value === 'string') {
        numVal = value;
    } else if (typeof value === 'number') {
        numVal = value.toFixed(1);
    }

    useInteractionEvent(lsk, () => {
        const newVal = parseFloat(scratchpad);
        if (!Number.isNaN(newVal)) {
            if (newVal >= min && newVal <= max) {
                selectedCallback(newVal);
            } else {
                setScratchpad('ENTRY OUT OF RANGE');
            }
        } else {
            setScratchpad('FORMAT ERROR');
        }
    });

    return (
        <span className={`${color} ${side} ${size}`}>{value === undefined ? nullValue : numVal}</span>
    );
};
