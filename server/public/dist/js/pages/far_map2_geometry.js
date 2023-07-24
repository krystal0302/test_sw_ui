// ==========================
//        Geometry
// ==========================
class Point2D {
	constructor(x, y) {
		this.x = Number(x);
		this.y = Number(y);
	}
}

class Line2D {
	constructor(p1, p2) {
		this.p1 = p1;
		this.p2 = p2;
	}
}

class Geometry {
	constructor() {
	}

	// 0: colinear, 1: clockwise, 2: counter-clockwise
	#rotateDirection(a, b, c) {
		const val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);

		if (val == 0)
			return 0;
		else if (val < 0)
			return 2;

		return 1;
	}

	pointOnLine(line, point) {
		// check whether point is on the line or not
		if (point.x <= Math.max(line.p1.x, line.p2.x)
			&& point.x <= Math.min(line.p1.x, line.p2.x)
			&& (point.y <= Math.max(line.p1.y, line.p2.y)
				&& point.y <= Math.min(line.p1.y, line.p2.y))) {
			return true;
		}
		return false;
	}

	lineIntersectLine(l1, l2) {
		// four direction for two lines and points of other line
		let dir1 = this.#rotateDirection(l1.p1, l1.p2, l2.p1);
		let dir2 = this.#rotateDirection(l1.p1, l1.p2, l2.p2);
		let dir3 = this.#rotateDirection(l2.p1, l2.p2, l1.p1);
		let dir4 = this.#rotateDirection(l2.p1, l2.p2, l1.p2);

		// intersecting
		if (dir1 != dir2 && dir3 != dir4)
			return true;

		// p2 of line2 are on the line1
		if (dir1 == 0 && this.pointOnLine(l1, l2.p1))
			return true;

		// p1 of line2 are on the line1
		if (dir2 == 0 && this.pointOnLine(l1, l2.p2))
			return true;

		// p2 of line1 are on the line2
		if (dir3 == 0 && this.pointOnLine(l2, l1.p1))
			return true;

		// p1 of line1 are on the line2
		if (dir4 == 0 && this.pointOnLine(l2, l1.p2))
			return true;

		return false;
	}

	polygonLines(poly) {
		let lines = [];
		let len = poly.length;
		for (let i = 0; i < len; i++) {
			let fromPoint = new Point2D(poly[(i % len)].x, poly[(i % len)].y);
			let toPoint = new Point2D(poly[((i + 1) % len)].x, poly[((i + 1) % len)].y);
			let line = new Line2D(fromPoint, toPoint);
			lines.push(line);
		}
		return lines;
	}

	/**
	* Check whether point is inside polygon
	* @param {json array, number, Point2D}
	* @returns {boolean}
	*/
	pointInPolygon(poly, n, p) {
		// create a point at infinity, y is same as point
		let tmp = new Point2D(9999, p.y);
		let exline = new Line2D(p, tmp);
		let count = 0;
		let i = 0;
		do {
			// forming a line from two consecutive points of polygon
			let fromPoint = new Point2D(poly[i].x, poly[i].y);
			let toPoint = new Point2D(poly[(i + 1) % n].x, poly[(i + 1) % n].y);
			let side = new Line2D(fromPoint, toPoint);
			if (this.lineIntersectLine(side, exline)) {
				// side is intersects exline
				if (this.#rotateDirection(side.p1, p, side.p2) == 0)
					return this.pointOnLine(side, p);
				count++;
			}
			i = (i + 1) % n;
		} while (i != 0);

		// count is odd
		return count & 1;
	}

}


function distanceBetweenPoints(p1, p2) {
	return Math.hypot(Math.abs(p2.y - p1.y), Math.abs(p2.x - p1.x));
}