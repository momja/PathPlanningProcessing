// Max Omdal 2020

class Cylinder {
    float radius;
    float height;
    Vec3 center;
    float sides = 20;
    Vec3 materialColor = new Vec3(255,255,255);
    PShape mesh;

    public Cylinder(float radius, float height, Vec3 center) {
        this.radius = radius;
        this.height = height;
        this.center = center;
        calculateMesh();
    }

    public void calculateMesh() {
        float theta = 0;
        float stepSize = 2*PI/sides;
        this.mesh = createShape(GROUP);
        PShape barrel = createShape();
        barrel.beginShape(TRIANGLES);
        barrel.noStroke();
        barrel.fill(materialColor.x, materialColor.y, materialColor.z);
        // Draw the sides of the cylinder
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            Vec3 vertexPos = new Vec3(this.center.x + this.radius * cos(theta),
                                    this.center.y + this.height/2,
                                    this.center.z + this.radius * sin(theta));
            Vec3 vertexPos2 = new Vec3(this.center.x + this.radius * cos(theta+stepSize),
                                       this.center.y + this.height/2,
                                       this.center.z + this.radius * sin(theta+stepSize));
            Vec3 vecNormal = center.minus(vertexPos).normalized();
            // barrel.normal(vecNormal.x, vecNormal.y, vecNormal.z);

            barrel.vertex(vertexPos.x, vertexPos.y, vertexPos.z);
            barrel.vertex(vertexPos.x, vertexPos.y - this.height, vertexPos.z);
            barrel.vertex(vertexPos2.x, vertexPos2.y - this.height, vertexPos2.z);
            
            barrel.vertex(vertexPos2.x, vertexPos2.y - this.height, vertexPos2.z);
            barrel.vertex(vertexPos2.x, vertexPos2.y, vertexPos2.z);
            barrel.vertex(vertexPos.x, vertexPos.y, vertexPos.z);
        }
        barrel.endShape(CLOSE);
        
        // Draw the caps
        PShape cap1 = createShape();
        theta = 0;
        cap1.beginShape(TRIANGLES);
        cap1.noStroke();
        cap1.fill(materialColor.x, materialColor.y, materialColor.z);
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            cap1.vertex(this.center.x + this.radius * cos(theta),
                   this.center.y + this.height/2,
                   this.center.z + this.radius * sin(theta));
            cap1.vertex(this.center.x + this.radius * cos(theta+stepSize),
                        this.center.y + this.height/2,
                        this.center.z + this.radius * sin(theta+stepSize));
            cap1.vertex(this.center.x, this.center.y + this.height/2, this.center.z);
        }
        cap1.endShape();

        PShape cap2 = createShape();
        theta = 0;
        cap2.beginShape(TRIANGLES);
        cap2.noStroke();
        cap2.fill(materialColor.x, materialColor.y, materialColor.z);
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            cap2.vertex(this.center.x + this.radius * cos(theta),
                   this.center.y - this.height/2,
                   this.center.z + this.radius * sin(theta));
            cap2.vertex(this.center.x + this.radius * cos(theta+stepSize),
                        this.center.y - this.height/2,
                        this.center.z + this.radius * sin(theta+stepSize));
            cap2.vertex(this.center.x, this.center.y - this.height/2, this.center.z);
        }
        cap2.endShape();

        this.mesh.addChild(barrel);
        this.mesh.addChild(cap1);
        this.mesh.addChild(cap2);
    }

    public void draw() {
        shape(this.mesh);
    }
}