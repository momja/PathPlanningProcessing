class OctreePoints {
    OctantPoints bounds;
    int capacity;
    ArrayList<Vec3> points;
    boolean divided = false;

    OctreePoints q111,q011,q001,q101,q110,q010,q000,q100;

    public OctreePoints(OctantPoints bounds, int capacity) {
        this.bounds = bounds;
        this.capacity = capacity;
        this.points = new ArrayList<Vec3>();
    }

    public void insert(Vec3 p) {
        if (!this.bounds.contains(p)) {
            return;
        }

        if (this.points.size() < this.capacity) {
            this.points.add(p);
        } else {
            if (!this.divided) {
                subdivide();
            }

            q111.insert(p);
            q011.insert(p);
            q001.insert(p);
            q101.insert(p);
            q110.insert(p);
            q010.insert(p);
            q000.insert(p);
            q100.insert(p);
        }
    }

    public void show() {
        strokeWeight(0.8);
        stroke(255);
        noFill();
        pushMatrix();
        translate(bounds.origin.x, bounds.origin.y, bounds.origin.z);
        box(bounds.size.x, bounds.size.y, bounds.size.z);
        popMatrix();
        if (this.divided) {
            q111.show();
            q011.show();
            q001.show();
            q101.show();
            q110.show();
            q010.show();
            q000.show();
            q100.show();
        }
        pushStyle();
        strokeWeight(4);
        stroke(255,0,0);
        for (Vec3 p : points) {
            // point(p.x, p.y, p.z);
        }
        popStyle();
    }

    public ArrayList<Vec3> rayIntersectsOctants(Ray3 ray) {
        // Gets all octants that the ray is in and returns the points stored in those octants
        
        ArrayList<Vec3> pointsInOctants = new ArrayList<Vec3>();
        
        if (bounds.rayIntersects(ray)) {
            if (divided) {
                // Recursively divide
                pointsInOctants.addAll(q111.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q011.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q001.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q101.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q110.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q010.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q000.rayIntersectsOctants(ray));
                pointsInOctants.addAll(q100.rayIntersectsOctants(ray));
            } else {
                // Just add the points in this oct
                pointsInOctants.addAll(points);
            }
        }

        return pointsInOctants;
    }

    private void subdivide() {
        Vec3 origin = bounds.origin;
        Vec3 size = bounds.size;
        Vec3 subdivideSize = size.times(0.5);

        OctantPoints q111_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5,0.5,0.5))), subdivideSize);
        q111 = new OctreePoints(q111_b, capacity);
        OctantPoints q011_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5,0.5,0.5))), subdivideSize);
        q011 = new OctreePoints(q011_b, capacity);
        OctantPoints q001_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5,-0.5,0.5))), subdivideSize);
        q001 = new OctreePoints(q001_b, capacity);
        OctantPoints q101_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5,-0.5,0.5))), subdivideSize);
        q101 = new OctreePoints(q101_b, capacity);
        OctantPoints q110_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5,0.5,-0.5))), subdivideSize);
        q110 = new OctreePoints(q110_b, capacity);
        OctantPoints q010_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5,0.5,-0.5))), subdivideSize);
        q010 = new OctreePoints(q010_b, capacity);
        OctantPoints q000_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(-0.5,-0.5,-0.5))), subdivideSize);
        q000 = new OctreePoints(q000_b, capacity);
        OctantPoints q100_b = new OctantPoints(origin.plus(subdivideSize.times(new Vec3(0.5,-0.5,-0.5))), subdivideSize);
        q100 = new OctreePoints(q100_b, capacity);

        this.divided = true;
    }

}