// Keyboard and gamepad share the camera tween, but release independently.
export const LOOK_DIRECTIONS = ['up', 'down', 'left', 'right'];

export function canLookAround() {
    const tag = document.activeElement?.tagName;
    return !document.hidden && document.hasFocus()
        && !['INPUT', 'TEXTAREA', 'SELECT'].includes(tag)
        && !document.activeElement?.isContentEditable
        && !!document.getElementById('tab-flight-data')?.classList.contains('active');
}

class LookAroundInput {
    constructor() {
        this.keyboard = {};
        this.gamepad = {};
        this.directions = {};
    }

    setKey(direction, pressed) {
        if (LOOK_DIRECTIONS.includes(direction)) this.keyboard[direction] = !!pressed;
    }

    setGamepad(directions) {
        for (const dir of LOOK_DIRECTIONS) this.gamepad[dir] = !!directions[dir];
    }

    release(source) {
        for (const dir of LOOK_DIRECTIONS) this[source][dir] = false;
    }

    read() {
        const allowed = canLookAround();
        for (const dir of LOOK_DIRECTIONS) {
            this.directions[dir] = allowed && !!(this.keyboard[dir] || this.gamepad[dir]);
        }
        return this.directions;
    }
}

export const lookAroundInput = new LookAroundInput();
