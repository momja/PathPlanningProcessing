class OctreeTriangles {
    OctantTris bounds;
    int capacity;
    ArrayList<PShape> tris;
    boolean divided = false;

    OctreeTriangles q111,q011,q001,q101,q110,q010,q000,q100;

    public OctreeTriangles(OctantTris bounds, int capacity) {
        this.bounds = bounds;
        this.capacity = capacity;
        this.tris = new ArrayList<PShape>();
    }

    public void insert(PShape tri) {
        if (!this.bounds.contains(tri)) {
            return;
        }

        if (!this.divided && this.tris.size() < this.capacity) {
            this.tris.add(tri);
        } else {
            if (!this.divided) {
                subdivide();
            }

            q111.insert(tri);
            q011.insert(tri);
            q001.insert(tri);
            q101.insert(tri);
            q110.insert(tri);
            q010.insert(tri);
            q000.insert(tri);
            q100.insert(tri);
        }
    }

    public void show(boolean includeTris) {
        bounds.show();
        if (this.divided) {
            q111.show(includeTris);
            q011.show(includeTris);
            q001.show(includeTris);
            q101.show(includeTris);
            q110.show(includeTris);
            q010.show(includeTris);
            q000.show(includeTris);
            q100.show(includeTris);
        }
        pushStyle();
        strokeWeight(4);
        stroke(255,0,0);
        if (includeTris) {
            for (PShape p : tris) {
                // shape(p);
            }
        }
        popStyle();
    }

    public ArrayList<PShape> rayIntersectsOctants(Ray3 ray) {
        // Gets all OctantTriss that the ray is in and returns the points stored in those OctantTriss
        ArrayList<PShape> pointsInOctantTris = new ArrayList<PShape>();
        
        if (bounds.rayIntersects(ray)) {
            if (divided) {
                // Recursively divide
                pointsInOctantTris.addAll(q111.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q011.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q001.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q101.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q110.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q010.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q000.rayIntersectsOctants(ray));
                pointsInOctantTris.addAll(q100.rayIntersectsOctants(ray));
            } else {
                // Just add the points in this oct
                pointsInOctantTris.addAll(tris);
            }
        }

        return pointsInOctantTris;
    }

    private void subdivide() {
        Vec3 origin = bounds.origin;
        Vec3 size = bounds.size;
        Vec3 subdivideSize = size.times(0.5);

        OctantTris q111_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5,0.5,0.5))), subdivideSize);
        q111 = new OctreeTriangles(q111_b, capacity);
        OctantTris q011_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5,0.5,0.5))), subdivideSize);
        q011 = new OctreeTriangles(q011_b, capacity);
        OctantTris q001_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5,-0.5,0.5))), subdivideSize);
        q001 = new OctreeTriangles(q001_b, capacity);
        OctantTris q101_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5,-0.5,0.5))), subdivideSize);
        q101 = new OctreeTriangles(q101_b, capacity);
        OctantTris q110_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5,0.5,-0.5))), subdivideSize);
        q110 = new OctreeTriangles(q110_b, capacity);
        OctantTris q010_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5,0.5,-0.5))), subdivideSize);
        q010 = new OctreeTriangles(q010_b, capacity);
        OctantTris q000_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(-0.5,-0.5,-0.5))), subdivideSize);
        q000 = new OctreeTriangles(q000_b, capacity);
        OctantTris q100_b = new OctantTris(origin.plus(subdivideSize.times(new Vec3(0.5,-0.5,-0.5))), subdivideSize);
        q100 = new OctreeTriangles(q100_b, capacity);

        // Insert all leaf nodes in children now
        for (PShape tri : tris) {
            q111.insert(tri);
            q011.insert(tri);
            q001.insert(tri);
            q101.insert(tri);
            q110.insert(tri);
            q010.insert(tri);
            q000.insert(tri);
            q100.insert(tri);
        }

        tris = new ArrayList<PShape>();

        this.divided = true;
    }

}