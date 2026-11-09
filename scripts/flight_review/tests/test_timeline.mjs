import assert from "node:assert/strict";
import { upperBound } from "../static/timeline.mjs";
assert.equal(upperBound([], 1), 0);
assert.equal(upperBound([1, 2, 2, 4], 0), 0);
assert.equal(upperBound([1, 2, 2, 4], 2), 3);
assert.equal(upperBound([1, 2, 2, 4], 3), 3);
assert.equal(upperBound([1, 2, 2, 4], 8), 4);
assert.equal(upperBound([[1], [4]], 2, r => r[0]), 1);
console.log("Timeline clipping tests passed");
