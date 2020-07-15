// Max Omdal 2020

class Cylinder {
    float radius;
    float height;
    Vec3 center;
    float sides = 20;
    Vec3 materialColor = new Vec3(255,255,255);

    public Cylinder(float radius, float height, Vec3 center) {
        this.radius = radius;
        this.height = height;
        this.center = center;
    }

    public void draw() {
        float theta = 0;
        float stepSize = 2*PI/sides;
        push();
        noStroke();
        fill(materialColor.x, materialColor.y, materialColor.z);
        // Draw the sides of the cylinder
        beginShape(QUAD_STRIP);
        for (int i = 0; i < sides+1; i++) {
            theta += stepSize;
            Vec3 vertexPos = new Vec3(this.center.x + this.radius * cos(theta),
                                    this.center.y + this.height/2,
                                    this.center.z + this.radius * sin(theta));
            Vec3 vecNormal = center.minus(vertexPos).normalized();
            normal(vecNormal.x, vecNormal.y, vecNormal.z);
            vertex(vertexPos.x, vertexPos.y, vertexPos.z);
            vertex(vertexPos.x, vertexPos.y - this.height, vertexPos.z);
        }
        endShape(CLOSE);
        
        // Draw the caps
        theta = 0;
        beginShape();
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            vertex(this.center.x + this.radius * cos(theta),
                   this.center.y + this.height/2,
                   this.center.z + this.radius * sin(theta));
        }
        endShape();

        theta = 0;
        beginShape();
        for (int i = 0; i < sides; i++) {
            theta += stepSize;
            vertex(this.center.x + this.radius * cos(theta),
                   this.center.y - this.height/2,
                   this.center.z + this.radius * sin(theta));
        }
        endShape();

        pop();
    }
}