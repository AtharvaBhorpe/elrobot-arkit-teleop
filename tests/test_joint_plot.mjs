import assert from "node:assert/strict";

import { frameFromX } from "../src/elrobot/web/static/joint-plot.mjs";

assert.equal(frameFromX(50, 50, 100, 11), 0);
assert.equal(frameFromX(100, 50, 100, 11), 5);
assert.equal(frameFromX(150, 50, 100, 11), 10);
assert.equal(frameFromX(-20, 50, 100, 11), 0);
assert.equal(frameFromX(900, 50, 100, 11), 10);
assert.equal(frameFromX(100, 50, 0, 11), 0);
assert.equal(frameFromX(100, 50, 100, 0), 0);
console.log("joint plot mapping: ok");
