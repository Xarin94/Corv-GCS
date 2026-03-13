/**
 * ExpressionParser.js - Safe Expression Evaluator for Flight Data
 * Parses and evaluates mathematical expressions with field references
 */

// Allowed mathematical functions (whitelist approach for security)
const MATH_FUNCTIONS = {
    sqrt: Math.sqrt,
    abs: Math.abs,
    sin: Math.sin,
    cos: Math.cos,
    tan: Math.tan,
    asin: Math.asin,
    acos: Math.acos,
    atan: Math.atan,
    atan2: Math.atan2,
    min: Math.min,
    max: Math.max,
    pow: Math.pow,
    log: Math.log,
    log10: Math.log10,
    exp: Math.exp,
    floor: Math.floor,
    ceil: Math.ceil,
    round: Math.round,
    sign: Math.sign
};

// Mathematical constants
const MATH_CONSTANTS = {
    PI: Math.PI,
    E: Math.E,
    RAD: 180 / Math.PI,  // Radians to degrees
    DEG: Math.PI / 180   // Degrees to radians
};

/**
 * Get nested value from object using dot notation path
 * @param {Object} obj - Source object
 * @param {string} path - Dot-separated path (e.g., 'state.lat')
 * @returns {*} Value at path or undefined
 */
function getNestedValue(obj, path) {
    if (!obj || !path) return undefined;
    const keys = path.split('.');
    let current = obj;
    for (const key of keys) {
        if (current === null || current === undefined) return undefined;
        current = current[key];
    }
    return current;
}

// Field path mappings from raw log data
// Maps field names to getter functions
const FIELD_ALIASES = {
    // State fields
    'state.lat': (entry) => getNestedValue(entry, 'state.lat') ?? entry.lat ?? 0,
    'state.lon': (entry) => getNestedValue(entry, 'state.lon') ?? entry.lon ?? 0,
    'state.alt': (entry) => getNestedValue(entry, 'state.alt') ?? entry.alt ?? 0,
    'state.vn': (entry) => getNestedValue(entry, 'state.vn') ?? entry.vn ?? 0,
    'state.ve': (entry) => getNestedValue(entry, 'state.ve') ?? entry.ve ?? 0,
    'state.vd': (entry) => getNestedValue(entry, 'state.vd') ?? entry.vd ?? 0,
    'state.roll': (entry) => getNestedValue(entry, 'state.roll') ?? entry.roll ?? 0,
    'state.pitch': (entry) => getNestedValue(entry, 'state.pitch') ?? entry.pitch ?? 0,
    'state.yaw': (entry) => getNestedValue(entry, 'state.yaw') ?? entry.yaw ?? 0,

    // Derived state fields (computed on-the-fly)
    'state.as': (entry) => {
        const vn = getNestedValue(entry, 'state.vn') ?? entry.vn ?? 0;
        const ve = getNestedValue(entry, 'state.ve') ?? entry.ve ?? 0;
        const vd = getNestedValue(entry, 'state.vd') ?? entry.vd ?? 0;
        return Math.sqrt(vn ** 2 + ve ** 2 + vd ** 2);
    },
    'state.gs': (entry) => {
        const vn = getNestedValue(entry, 'state.vn') ?? entry.vn ?? 0;
        const ve = getNestedValue(entry, 'state.ve') ?? entry.ve ?? 0;
        return Math.sqrt(vn ** 2 + ve ** 2);
    },
    'state.vs': (entry) => {
        const vd = getNestedValue(entry, 'state.vd') ?? entry.vd ?? 0;
        return -vd;
    },

    // IMU fields
    'imu.ax': (entry) => getNestedValue(entry, 'imu.ax') ?? entry.ax ?? 0,
    'imu.ay': (entry) => getNestedValue(entry, 'imu.ay') ?? entry.ay ?? 0,
    'imu.az': (entry) => getNestedValue(entry, 'imu.az') ?? entry.az ?? 0,
    'imu.gx': (entry) => getNestedValue(entry, 'imu.gx') ?? entry.gx ?? 0,
    'imu.gy': (entry) => getNestedValue(entry, 'imu.gy') ?? entry.gy ?? 0,
    'imu.gz': (entry) => getNestedValue(entry, 'imu.gz') ?? entry.gz ?? 0
};

/**
 * Custom error class for expression parsing
 */
export class ExpressionError extends Error {
    constructor(message) {
        super(message);
        this.name = 'ExpressionError';
    }
}

/**
 * Tokenize an expression string
 * @param {string} expr - Expression to tokenize
 * @returns {Array} Array of tokens
 */
function tokenize(expr) {
    const tokens = [];
    let i = 0;
    const len = expr.length;

    while (i < len) {
        const char = expr[i];

        // Skip whitespace
        if (/\s/.test(char)) {
            i++;
            continue;
        }

        // Number (including decimals and scientific notation)
        if (/[0-9.]/.test(char)) {
            let num = '';
            while (i < len && /[0-9.eE+-]/.test(expr[i])) {
                // Handle scientific notation sign carefully
                if ((expr[i] === '+' || expr[i] === '-') &&
                    num.length > 0 &&
                    !/[eE]$/.test(num)) {
                    break;
                }
                num += expr[i++];
            }
            const parsed = parseFloat(num);
            if (isNaN(parsed)) {
                throw new ExpressionError(`Invalid number: '${num}'`);
            }
            tokens.push({ type: 'NUMBER', value: parsed });
            continue;
        }

        // Identifier (function, field, or constant)
        if (/[a-zA-Z_]/.test(char)) {
            let ident = '';
            while (i < len && /[a-zA-Z0-9_.]/.test(expr[i])) {
                ident += expr[i++];
            }
            tokens.push({ type: 'IDENT', value: ident });
            continue;
        }

        // Operators and punctuation
        if ('+-*/^(),%'.includes(char)) {
            tokens.push({ type: 'OP', value: char });
            i++;
            continue;
        }

        // Unknown character
        throw new ExpressionError(`Unexpected character: '${char}' at position ${i}`);
    }

    return tokens;
}

/**
 * Parse and compile an expression into an evaluator function
 * Uses a simple recursive descent parser
 * @param {string} expr - Expression string
 * @returns {Function} Evaluator function that takes a log entry and returns a number
 */
export function compileExpression(expr) {
    if (!expr || typeof expr !== 'string') {
        throw new ExpressionError('Expression must be a non-empty string');
    }

    expr = expr.trim();
    if (!expr) {
        throw new ExpressionError('Expression cannot be empty');
    }

    const tokens = tokenize(expr);
    let pos = 0;

    function peek() {
        return tokens[pos] || null;
    }

    function consume(expectedType, expectedValue) {
        const token = tokens[pos];
        if (!token) {
            throw new ExpressionError('Unexpected end of expression');
        }
        if (expectedType && token.type !== expectedType) {
            throw new ExpressionError(`Expected ${expectedType} but got ${token.type}`);
        }
        if (expectedValue && token.value !== expectedValue) {
            throw new ExpressionError(`Expected '${expectedValue}' but got '${token.value}'`);
        }
        pos++;
        return token;
    }

    // Grammar:
    // expr     -> term (('+' | '-') term)*
    // term     -> power (('*' | '/') power)*
    // power    -> unary ('^' unary)?
    // unary    -> '-' unary | primary
    // primary  -> NUMBER | IDENT | IDENT '(' args ')' | '(' expr ')'
    // args     -> expr (',' expr)*

    function parseExpr() {
        let left = parseTerm();

        while (peek() && peek().type === 'OP' && (peek().value === '+' || peek().value === '-')) {
            const op = consume('OP').value;
            const right = parseTerm();
            const leftFn = left;
            const rightFn = right;
            if (op === '+') {
                left = (entry) => leftFn(entry) + rightFn(entry);
            } else {
                left = (entry) => leftFn(entry) - rightFn(entry);
            }
        }

        return left;
    }

    function parseTerm() {
        let left = parsePower();

        while (peek() && peek().type === 'OP' && (peek().value === '*' || peek().value === '/')) {
            const op = consume('OP').value;
            const right = parsePower();
            const leftFn = left;
            const rightFn = right;
            if (op === '*') {
                left = (entry) => leftFn(entry) * rightFn(entry);
            } else {
                left = (entry) => {
                    const divisor = rightFn(entry);
                    if (divisor === 0) return NaN;
                    return leftFn(entry) / divisor;
                };
            }
        }

        return left;
    }

    function parsePower() {
        let base = parseUnary();

        if (peek() && peek().type === 'OP' && peek().value === '^') {
            consume('OP', '^');
            const exp = parseUnary();
            const baseFn = base;
            const expFn = exp;
            return (entry) => Math.pow(baseFn(entry), expFn(entry));
        }

        return base;
    }

    function parseUnary() {
        if (peek() && peek().type === 'OP' && peek().value === '-') {
            consume('OP', '-');
            const operand = parseUnary();
            return (entry) => -operand(entry);
        }
        return parsePrimary();
    }

    function parsePrimary() {
        const token = peek();

        if (!token) {
            throw new ExpressionError('Unexpected end of expression');
        }

        // Number literal
        if (token.type === 'NUMBER') {
            consume('NUMBER');
            const value = token.value;
            return () => value;
        }

        // Identifier (field, function, or constant)
        if (token.type === 'IDENT') {
            const ident = consume('IDENT').value;

            // Check if it's a function call
            if (peek() && peek().type === 'OP' && peek().value === '(') {
                consume('OP', '(');

                // Parse arguments
                const args = [];
                if (!(peek() && peek().type === 'OP' && peek().value === ')')) {
                    args.push(parseExpr());
                    while (peek() && peek().type === 'OP' && peek().value === ',') {
                        consume('OP', ',');
                        args.push(parseExpr());
                    }
                }
                consume('OP', ')');

                // Look up function
                const func = MATH_FUNCTIONS[ident.toLowerCase()];
                if (!func) {
                    throw new ExpressionError(`Unknown function: '${ident}'`);
                }

                return (entry) => func(...args.map(arg => arg(entry)));
            }

            // Check if it's a constant
            const upperIdent = ident.toUpperCase();
            if (MATH_CONSTANTS.hasOwnProperty(upperIdent)) {
                const value = MATH_CONSTANTS[upperIdent];
                return () => value;
            }

            // Must be a field reference
            const lowerIdent = ident.toLowerCase();
            const fieldGetter = FIELD_ALIASES[lowerIdent] || FIELD_ALIASES[ident];
            if (fieldGetter) {
                return fieldGetter;
            }

            // Unknown identifier
            throw new ExpressionError(`Unknown field or constant: '${ident}'`);
        }

        // Parenthesized expression
        if (token.type === 'OP' && token.value === '(') {
            consume('OP', '(');
            const inner = parseExpr();
            consume('OP', ')');
            return inner;
        }

        throw new ExpressionError(`Unexpected token: '${token.value}'`);
    }

    const evaluator = parseExpr();

    // Check for leftover tokens
    if (pos < tokens.length) {
        throw new ExpressionError(`Unexpected token after expression: '${tokens[pos].value}'`);
    }

    // Return a safe wrapper that handles errors gracefully
    return (entry) => {
        try {
            const result = evaluator(entry);
            return Number.isFinite(result) ? result : NaN;
        } catch (e) {
            return NaN;
        }
    };
}

/**
 * Validate an expression without compiling it fully
 * @param {string} expr - Expression to validate
 * @returns {{ valid: boolean, error?: string }} Validation result
 */
export function validateExpression(expr) {
    try {
        compileExpression(expr);
        return { valid: true };
    } catch (e) {
        return {
            valid: false,
            error: e instanceof ExpressionError ? e.message : 'Invalid expression'
        };
    }
}

/**
 * Get list of available fields for autocomplete/help
 * @returns {Array<{name: string, description: string}>}
 */
export function getAvailableFields() {
    return [
        { name: 'state.lat', description: 'Latitude (degrees)' },
        { name: 'state.lon', description: 'Longitude (degrees)' },
        { name: 'state.alt', description: 'Altitude (meters)' },
        { name: 'state.vn', description: 'North velocity (m/s)' },
        { name: 'state.ve', description: 'East velocity (m/s)' },
        { name: 'state.vd', description: 'Down velocity (m/s)' },
        { name: 'state.roll', description: 'Roll angle (radians)' },
        { name: 'state.pitch', description: 'Pitch angle (radians)' },
        { name: 'state.yaw', description: 'Yaw/Heading (radians)' },
        { name: 'state.as', description: 'Airspeed computed (m/s)' },
        { name: 'state.gs', description: 'Ground speed computed (m/s)' },
        { name: 'state.vs', description: 'Vertical speed computed (m/s)' },
        { name: 'imu.ax', description: 'Acceleration X (m/s^2)' },
        { name: 'imu.ay', description: 'Acceleration Y (m/s^2)' },
        { name: 'imu.az', description: 'Acceleration Z (m/s^2)' },
        { name: 'imu.gx', description: 'Gyro X (rad/s)' },
        { name: 'imu.gy', description: 'Gyro Y (rad/s)' },
        { name: 'imu.gz', description: 'Gyro Z (rad/s)' }
    ];
}

/**
 * Get list of available functions for help
 * @returns {Array<{name: string, description: string, example: string}>}
 */
export function getAvailableFunctions() {
    return [
        { name: 'sqrt(x)', description: 'Square root', example: 'sqrt(imu.ax^2 + imu.ay^2)' },
        { name: 'abs(x)', description: 'Absolute value', example: 'abs(state.vs)' },
        { name: 'sin(x)', description: 'Sine (x in radians)', example: 'sin(state.roll)' },
        { name: 'cos(x)', description: 'Cosine (x in radians)', example: 'cos(state.pitch)' },
        { name: 'tan(x)', description: 'Tangent', example: 'tan(state.roll)' },
        { name: 'asin(x)', description: 'Arc sine', example: 'asin(0.5) * RAD' },
        { name: 'acos(x)', description: 'Arc cosine', example: 'acos(0.5)' },
        { name: 'atan(x)', description: 'Arc tangent', example: 'atan(state.vd/state.gs)' },
        { name: 'atan2(y,x)', description: 'Arc tangent of y/x', example: 'atan2(state.ve, state.vn)' },
        { name: 'min(a,b)', description: 'Minimum of two values', example: 'min(state.as, 100)' },
        { name: 'max(a,b)', description: 'Maximum of two values', example: 'max(state.alt, 0)' },
        { name: 'pow(x,y)', description: 'x raised to power y', example: 'pow(imu.ax, 2)' },
        { name: 'log(x)', description: 'Natural logarithm', example: 'log(state.alt)' },
        { name: 'log10(x)', description: 'Base-10 logarithm', example: 'log10(state.alt)' },
        { name: 'exp(x)', description: 'Exponential (e^x)', example: 'exp(-state.alt/8000)' },
        { name: 'floor(x)', description: 'Round down', example: 'floor(state.alt)' },
        { name: 'ceil(x)', description: 'Round up', example: 'ceil(state.as)' },
        { name: 'round(x)', description: 'Round to nearest', example: 'round(state.alt)' },
        { name: 'sign(x)', description: 'Sign (-1, 0, or 1)', example: 'sign(state.vs)' }
    ];
}

/**
 * Get available constants
 * @returns {Array<{name: string, value: number, description: string}>}
 */
export function getAvailableConstants() {
    return [
        { name: 'PI', value: Math.PI, description: 'Pi (3.14159...)' },
        { name: 'E', value: Math.E, description: "Euler's number (2.71828...)" },
        { name: 'RAD', value: 180 / Math.PI, description: 'Radians to degrees multiplier' },
        { name: 'DEG', value: Math.PI / 180, description: 'Degrees to radians multiplier' }
    ];
}

export default {
    compileExpression,
    validateExpression,
    ExpressionError,
    getAvailableFields,
    getAvailableFunctions,
    getAvailableConstants
};
