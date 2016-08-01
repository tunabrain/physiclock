"use strict";

/* Disclaimer: This is *not* how you should do physics.

Copyright (c) 2016 Benedikt Bitterli

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from
the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
   that you wrote the original software. If you use this software in a product,
   an acknowledgment in the product documentation would be appreciated but is
   not required.

2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.

*/

var CollisionInfo = function() {
    this.reset();
}
CollisionInfo.prototype.reset = function() {
    this.x = this.y = this.nx = this.ny = 0;
    this.depth = Infinity;
}
CollisionInfo.prototype.update = function(depth, nx, ny, x, y) {
    if (depth < this.depth) {
        this.x = x;
        this.y = y;
        this.nx = nx;
        this.ny = ny;
        this.depth = depth;
        return 1;
    }
    return 0;
}

var Body = function(type) {
    this.type = type;
    this.visible = true;
}
Body.GRAVITY = 1;

Body.SPHERE = 0;
Body.LINE = 1;
Body.SEESAW = 2;

Body.prototype.sphereSphereCollision = function(other, collisionInfo) {
    var dx = other.x - this.x;
    var dy = other.y - this.y;
    var dSq = dx*dx + dy*dy;
    if (dSq < (this.radius + other.radius)*(this.radius + other.radius)) {
        var d = Math.sqrt(dSq);
        return collisionInfo.update(this.radius + other.radius - d, dx/d, dy/d, this.x, this.y);
    }
    return 0;
}
Body.prototype.lineSphereCollision = function(other, collisionInfo) {
    var dx =   other.x - this.s1.x, dy =   other.y - this.s1.y;
    var lx = this.s2.x - this.s1.x, ly = this.s2.y - this.s1.y;
    var t = (dx*lx + dy*ly)/(lx*lx + ly*ly);
    
    if (t < 0.0) {
        return this.s1.sphereSphereCollision(other, collisionInfo);
    } else if (t > 1.0) {
        return this.s2.sphereSphereCollision(other, collisionInfo);
    } else {
        var l = Math.sqrt(lx*lx + ly*ly);
        var nx = -ly/l;
        var ny = lx/l;
        var dot = nx*dx + ny*dy;
        if (dot < 0.0) {
            dot = -dot;
            nx = -nx;
            ny = -ny;
        }
        if (dot < this.radius + other.radius)
            return collisionInfo.update(this.radius + other.radius - dot, nx, ny, other.x, other.y);
    }
    return 0;
}
Body.prototype.lineLineCollision = function(other, collisionInfo) {
    var os1 = this.lineSphereCollision(other.s1, collisionInfo);
    var os2 = this.lineSphereCollision(other.s2, collisionInfo);
    var s1o = other.lineSphereCollision(this.s1, collisionInfo);
    var s2o = other.lineSphereCollision(this.s2, collisionInfo);
    return (s1o || s2o) ? -1 : (os1 || os2) ? 1 : 0;
}
Body.prototype.detectCollision = function(other, collisionInfo) {
    if (this.type == Body.SPHERE) {
        if (other.type == Body.SPHERE)
            return this.sphereSphereCollision(other, collisionInfo);
        else
            return -other.lineSphereCollision(this, collisionInfo);
    } else {
        if (other.type == Body.SPHERE)
            return this.lineSphereCollision(other, collisionInfo);
        else
            return this.lineLineCollision(other, collisionInfo);
    }
}
Body.prototype.computeMass = function(info) {
    if (this.type == Body.SPHERE) {
        return this.mass;
    } else {
        var dx = info.x - this.s1.x, dy =    info.y - this.s1.y;
        var lx = this.s2.x - this.s1.x, ly = this.s2.y - this.s1.y;
        var t = (dx*lx + dy*ly)/(lx*lx + ly*ly);
        return (1.0 - t)*this.s1.mass + t*this.s2.mass;
    }
}
Body.prototype.applyImpulse = function(info, scale, isStatic) {
    if (this.type == Body.SPHERE) {
        this.x -= info.nx*info.depth*scale;
        this.y -= info.ny*info.depth*scale;
        if (isStatic) {
            var vx = this.x - this.oldX;
            var vy = this.y - this.oldY;
            var dot = Math.min(vx*info.nx*scale + vy*info.ny*scale, 0);
            vx -= info.nx*scale*dot;
            vy -= info.ny*scale*dot;
            this.oldX = this.x - vx;
            this.oldY = this.y - vy;
        }
    } else if (this.type == Body.SEESAW) {
        var dx =    info.x - this.s1.x, dy =    info.y - this.s1.y;
        var lx = this.s2.x - this.s1.x, ly = this.s2.y - this.s1.y;
        var t = (dx*lx + dy*ly)/(lx*lx + ly*ly);
        var r = this.s1.mass/(this.s1.mass + this.s2.mass);
        this.s1.applyImpulse(info, scale*(1.0 - t)*(1.0 - r));
        this.s2.applyImpulse(info, scale*t*r);
        this.updateSeesawAlignment();
    }
}
Body.prototype.updateSeesawAlignment = function() {
    var hingeX = this.s1.x*(1.0 - this.hinge) + this.s2.x*this.hinge;
    var hingeY = this.s1.y*(1.0 - this.hinge) + this.s2.y*this.hinge;
    var dx = (this.hingeX - hingeX)*this.lambda;
    var dy = (this.hingeY - hingeY)*this.lambda;
    var r = this.s1.mass/(this.s1.mass + this.s2.mass);
    this.s1.x += dx*(1.0 - this.hinge)*(1.0 - r);
    this.s1.y += dy*(1.0 - this.hinge)*(1.0 - r);
    this.s2.x += dx*this.hinge*r;
    this.s2.y += dy*this.hinge*r;
    
    var lx = this.s2.x - this.s1.x;
    var ly = this.s2.y - this.s1.y;
    var l = Math.sqrt(lx*lx + ly*ly);
    var dl = (l - this.length)/l*0.5;
    
    this.s1.x += lx*dl;
    this.s1.y += ly*dl;
    this.s2.x -= lx*dl;
    this.s2.y -= ly*dl;
}
Body.prototype.update = function() {
    if (this.type == Body.SPHERE) {
        var oldX = this.x, oldY = this.y;
        this.x += this.x - this.oldX;
        this.y += this.y - this.oldY;
        this.oldX = oldX;
        this.oldY = oldY - Body.GRAVITY;
    } else if (this.type == Body.SEESAW) {
        this.s1.update();
        this.s2.update();
        this.updateSeesawAlignment();
    }
}
Body.prototype.draw = function(ctx) {
    ctx.beginPath();
    ctx.fillStyle = "#ccc";
    ctx.strokeStyle = "#ccc";
    if (this.type == Body.SPHERE) {
        ctx.ellipse(this.x, this.y, this.radius, this.radius, 0.0, 0.0, Math.PI*2.0, false);
        ctx.fill();
    } else {
        ctx.lineCap = "round";
        ctx.lineWidth = this.radius*2.0;
        ctx.moveTo(this.s1.x, this.s1.y);
        ctx.lineTo(this.s2.x, this.s2.y);
        ctx.stroke();
    }
}
function Sphere(x, y, radius, mass) {
    var body = new Body(Body.SPHERE);
    body.oldX = body.x = x;
    body.oldY = body.y = y;
    body.radius = radius;
    body.mass = mass;
    return body;
}
function Line(x1, y1, x2, y2, radius) {
    var body = new Body(Body.LINE);
    body.s1 = Sphere(x1, y1, radius, 1.0);
    body.s2 = Sphere(x2, y2, radius, 1.0);
    body.radius = radius;
    return body;
}
function Seesaw(x1, y1, x2, y2, radius, hinge, mass1, mass2) {
    var body = new Body(Body.SEESAW);
    var ratio = mass1/(mass1 + mass2);
    body.lambda = 1.0/(hinge*hinge*ratio + (1.0 - hinge)*(1.0 - hinge)*(1.0 - ratio));
    body.s1 = Sphere(x1, y1, radius, mass1);
    body.s2 = Sphere(x2, y2, radius, mass2);
    body.hinge = hinge;
    body.hingeX = x1*(1.0 - hinge) + x2*hinge;
    body.hingeY = y1*(1.0 - hinge) + y2*hinge;
    body.length = Math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
    body.radius = radius;
    return body;
}

var Elevator = function(clock, ratio, x, y, width, height) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    
    this.line1 = Line(0, 0, 10, 10, 3);
    this.line2 = Line(10, 10, 20, 0, 3);
    this.s1 = Sphere(x + width*0.5, y + width*0.5, width*0.5, 1.0);
    this.s2 = Sphere(x + width*0.5, y + height - width*0.5, width*0.5, 1.0);
    clock.addStaticBody(this.line1);
    clock.addStaticBody(this.line2);
    clock.addStaticBody(this.s1);
    clock.addStaticBody(this.s2);
    
    this.setRatio(ratio);
    
    if (ratio > 0.5)
        clock.addDynamicBody(Sphere(0.5*(this.line1.s1.x + this.line2.s1.x), 0.5*(this.line1.s1.y + this.line2.s1.y) - 8, 10, 0.1));
}
Elevator.prototype.setRatio = function(ratio) {
    var circumCircle = this.width*Math.PI;
    var circleRatio = circumCircle/(circumCircle + (this.height - this.width)*2);
    var ropeRatio = 1.0 - circleRatio;
    
    if (ratio < 0.25*circleRatio) {
        var angle = Math.PI*0.5*(1.0 - ratio/(0.25*circleRatio));
        this.setCircleRatio(angle, this.s1.x, this.s1.y);
    } else if (ratio >= ropeRatio + circleRatio*0.75) {
        var angle = -Math.PI*(1.0 + 0.5*(ratio - ropeRatio - circleRatio*0.75)/(0.25*circleRatio));
        this.setCircleRatio(angle, this.s1.x, this.s1.y);
    } else if (ratio >= ropeRatio*0.5 + 0.25*circleRatio && ratio <= ropeRatio*0.5 + 0.75*circleRatio) {
        var angle = -Math.PI*(ratio - ropeRatio*0.5 - 0.25*circleRatio)/(0.5*circleRatio);
        this.setCircleRatio(angle, this.s2.x, this.s2.y);
    } else if (ratio >= 0.25*circleRatio && ratio <= 0.25*circleRatio + ropeRatio*0.5) {
        this.setRightRopeRatio((ratio - 0.25*circleRatio)/(0.5*ropeRatio));
    } else {
        this.setLeftRopeRatio((ratio - 0.5*ropeRatio - 0.75*circleRatio)/(0.5*ropeRatio));
    }
}
Elevator.prototype.setLeftRopeRatio = function(ratio) {
    var y = this.y - this.width*0.5 + this.height - (this.height - this.width)*ratio;
    this.line1.s1.x = this.x;      this.line1.s1.y = y;
    this.line1.s2.x = this.x - 10; this.line1.s2.y = y + 10;
    this.line2.s2.x = this.x - 10; this.line2.s2.y = y + 10;
    this.line2.s1.x = this.x - 20; this.line2.s1.y = y;
}
Elevator.prototype.setRightRopeRatio = function(ratio) {
    var y = this.y + this.width*0.5 + (this.height - this.width)*ratio;
    this.line1.s1.x = this.x + this.width;      this.line1.s1.y = y;
    this.line1.s2.x = this.x + this.width + 10; this.line1.s2.y = y - 10;
    this.line2.s2.x = this.x + this.width + 10; this.line2.s2.y = y - 10;
    this.line2.s1.x = this.x + this.width + 20; this.line2.s1.y = y;
}

Elevator.prototype.setCircleRatio = function(angle, x, y) {
    var d1 = this.width*0.5;
    var d2 = Math.sqrt((d1 + 10.0)*(d1 + 10.0) + 100.0);
    var d3 = d1 + 20.0;
    var delta = -Math.atan2(-10, d1 + 10);

    var x1 = x + d1*Math.cos(angle        ), y1 = y - d1*Math.sin(angle);
    var x2 = x + d2*Math.cos(angle + delta), y2 = y - d2*Math.sin(angle + delta);
    var x3 = x + d3*Math.cos(angle        ), y3 = y - d3*Math.sin(angle);

    this.line1.s1.x = x1; this.line1.s1.y = y1;
    this.line1.s2.x = x2; this.line1.s2.y = y2;
    this.line2.s2.x = x2; this.line2.s2.y = y2;
    this.line2.s1.x = x3; this.line2.s1.y = y3;
}
Elevator.prototype.drawBackground = function(ctx) {
    ctx.strokeStyle = "#eee";
    ctx.lineCap = "square";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(this.x + this.width - 2, this.y + this.width*0.5);
    ctx.lineTo(this.x + this.width - 2, this.y + this.height - this.width*0.5);
    ctx.moveTo(this.x + 2, this.y + this.width*0.5);
    ctx.lineTo(this.x + 2, this.y + this.height - this.width*0.5);
    ctx.stroke();
}
Elevator.prototype.drawForeground = function(ctx, seconds) {
    var circumference = this.width*Math.PI + this.height*2 - this.width*2;
    var turnPerSecond = Math.PI*2.0*circumference/(this.width*Math.PI*60.0);
    
    var rotation = seconds*turnPerSecond;
    var numSpokes = 10;
    
    for (var wheel = 0; wheel < 2; ++wheel) {
        ctx.save();
        ctx.translate(this.x + this.width*0.5, this.y + this.width*0.5 + wheel*(this.height - this.width));
        ctx.rotate(rotation);
        for (var i = 0; i < numSpokes; ++i) {
            ctx.strokeStyle = "#fff";
            ctx.lineCap = "round";
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(10, 0);
            ctx.lineTo(this.width*0.5 - 5, 0);
            ctx.stroke();
            ctx.rotate(Math.PI*2.0/numSpokes);
        }
        ctx.restore();
    }
}

var Clock = function(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.iterations = 20;
    this.speed = 1;
    this.lastTime = Date.now();
    
    this.setTime(new Date());
    this.reset();
    
    if (!this.ctx.ellipse) {
        this.ctx.ellipse = function(cx, cy, rx, ry, rot, aStart, aEnd) {
            this.save();
            this.translate(cx, cy);
            this.rotate(rot);
            this.translate(-rx, -ry);
            this.scale(rx, ry);
            this.arc(1, 1, 1, aStart, aEnd, false);
            this.restore();
        }
    }
}
Clock.prototype.addDynamicBody = function(body) {
    this.dynamicBodies.push(body);
}
Clock.prototype.addStaticBody = function(body) {
    this.staticBodies.push(body);
}
Clock.prototype.curDate = function() {
    return new Date(this.baseTime + (Date.now() - this.baseDate)*this.speed);
}
Clock.prototype.seconds = function() {
    var date = this.curDate();
    return date.getSeconds() + date.getMilliseconds()*1e-3;
}
Clock.prototype.reset = function() {
    this.staticBodies = [];
    this.dynamicBodies = [];
    
    this.addStaticBody(Line( 90,   407,   400,  397,   7));
    this.addStaticBody(Line( 67,   397,   80,   405,   3));
    this.addStaticBody(Line(-7,    383,   10,   405,   7));
    this.addStaticBody(Line( 50,   377,   367,  365,   3));
    this.addStaticBody(Line( 90,   363,   367,  365,   3));
    this.addStaticBody(Line( 90,   363,   23,   250,   3));
    this.addStaticBody(Line( 410,  360,   395,  395,   10));
    this.addStaticBody(Line( 100,  104.5, 250,  123,   3));
    this.addStaticBody(Line( 390,  145,   400,  145,   3));
    this.addStaticBody(Line( 170,  175,   290,  167,   3));
    this.addStaticBody(Line( 110,  197,   190,  207,   3));
    this.addStaticBody(Line( 390,  237,   400,  237,   3));
    this.addStaticBody(Line( 160,  260,   200,  256,   3));
    this.addStaticBody(Line( 250,  246,   250,  246,   3));
    this.addStaticBody(Line( 110,  336,   160,  335,   3));
    this.addStaticBody(Line( 260,  324,   260,  324,   3));
    this.addStaticBody(Line( 110,  280,   150,  285.5, 3));
    this.addStaticBody(Line( 390,  320,   400,  320,   3));
    this.addStaticBody(Line( 0,   -10,    400, -10,    10));
    this.addStaticBody(Line( 0,    410,   400,  410,   10));
    this.addStaticBody(Line(-12,   0,    -12,   410,   10));
    this.addStaticBody(Line( 410,  0,     410,  400,   10));
    
    this.addDynamicBody(Seesaw(260, 124, 390, 139, 3, 0.80, 1.0, 4.2));
    this.addDynamicBody(Seesaw(200, 212, 390, 230, 3, 0.80, 1.0, 4.5));
    this.addDynamicBody(Seesaw(163, 291, 390, 313, 3, 0.80, 1.0, 4.6));
    
    var date = this.curDate();
    var seesaw1 = date.getMinutes() % 6;
    var seesaw2 = Math.floor(date.getMinutes()/6);
    var seesaw3 = date.getHours() % 12;
    
    for (var i = 0; i < seesaw1; ++i)
        this.addDynamicBody(Sphere(365 - i*20, 122 - i*2, 10, 0.1));
    for (var i = 0; i < seesaw2; ++i)
        this.addDynamicBody(Sphere(390 - i*20, 216 - i*2, 10, 0.1));
    for (var i = 0; i < seesaw3; ++i)
        this.addDynamicBody(Sphere(390 - i*20, 300 - i*2, 10, 0.1));
    
    var restCount = 5 - seesaw1 + 9 - seesaw2 + 11 - seesaw3 + 2;
    
    for (var i = 0; i < Math.min(restCount, 14); ++i)
        this.addDynamicBody(Sphere(80 + i*21, 390 - i, 10, 0.1));
    for (var i = 0; i < restCount - 14; ++i)
        this.addDynamicBody(Sphere(373 - i*21, 350, 10, 0.1));
    
    this.elevator = new Elevator(this, this.seconds()/60.0, 20, 50, 60, 330);
}
Clock.prototype.update = function() {
    var curTime = Date.now();
    if (curTime - this.lastTime > 500)
        this.reset();
    this.lastTime = curTime;
    
    this.elevator.setRatio(this.seconds()/60.0);
    
    for (var i = 0; i < this.dynamicBodies.length; ++i)
        this.dynamicBodies[i].update();
    
    var collisionInfo = new CollisionInfo();
    for (var iter = 0; iter < this.iterations; ++iter) {
        for (var i = 0; i < this.dynamicBodies.length; ++i) {
            for (var j = 0; j < this.staticBodies.length; ++j) {
                collisionInfo.reset();
                var sign = this.dynamicBodies[i].detectCollision(this.staticBodies[j], collisionInfo);
                if (sign)
                    this.dynamicBodies[i].applyImpulse(collisionInfo, sign, false);
            }
        }

        for (var i = 0; i < this.dynamicBodies.length; ++i) {
            for (var j = i + 1; j < this.dynamicBodies.length; ++j) {
                collisionInfo.reset();
                var sign = this.dynamicBodies[i].detectCollision(this.dynamicBodies[j], collisionInfo);
                if (sign) {
                    var mass1 = this.dynamicBodies[i].computeMass(collisionInfo);
                    var mass2 = this.dynamicBodies[j].computeMass(collisionInfo);
                
                    var ratio = mass1/(mass1 + mass2);
                    
                    this.dynamicBodies[i].applyImpulse(collisionInfo, sign*(1.0 - ratio), false);
                    this.dynamicBodies[j].applyImpulse(collisionInfo, -sign*ratio, false);
                }
            }
        }
    }
}
Clock.prototype.draw = function() {
    var rect = this.canvas.getBoundingClientRect();
    this.canvas.width = rect.width;
    this.canvas.height = rect.height;
    
    var scale = Math.min(rect.width, rect.height)/415.0;
    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.translate(rect.width/2 - 200.0*scale, rect.height/2 - 200.0*scale)
    this.ctx.scale(scale, scale);

    this.ctx.fillStyle = "#fff";
    this.ctx.beginPath();
    this.ctx.rect(0, 0, 400, 400);
    this.ctx.fill();
    
    this.ctx.strokeStyle = "#ccc";
    this.ctx.lineCap = "square";
    this.ctx.lineWidth = 20;
    this.ctx.beginPath();
    this.ctx.moveTo(-12, -10);
    this.ctx.lineTo(410, -10);
    this.ctx.lineTo(410, 410);
    this.ctx.lineTo(-12, 410);
    this.ctx.lineTo(-12, -10);
    this.ctx.stroke();
    
    this.elevator.drawBackground(this.ctx, this.seconds());

    for (var i = 0; i < this.dynamicBodies.length; ++i)
        this.dynamicBodies[i].draw(this.ctx);
    for (var i = 0; i < this.staticBodies.length; ++i)
        this.staticBodies[i].draw(this.ctx);
        
    this.elevator.drawForeground(this.ctx, this.curDate().getTime()*1e-3 % 10000.0);
    
    this.ctx.textAlign = "right";
    this.fillStyle = "#000";
    this.ctx.save();
    this.ctx.translate(385, 150);
    this.ctx.rotate(6.5*Math.PI/180.0);
    for (var i = 0; i < 6; ++i)
        this.ctx.fillText((i + 1).toString(), -i*20 + (i ? 3 : 0), 0);
    this.ctx.restore();
    this.ctx.save();
    this.ctx.translate(385, 242);
    this.ctx.rotate(6.5*Math.PI/180.0);
    for (var i = 0; i < 10; ++i)
        this.ctx.fillText(((i + 1)*6).toString(), -i*20 + (i ? 7 : 0), 0);
    this.ctx.restore();
    this.ctx.save();
    this.ctx.translate(385, 325);
    this.ctx.rotate(6.5*Math.PI/180.0);
    for (var i = 0; i < 12; ++i)
        this.ctx.fillText((i + 1).toString(), -i*20 + (i ? 5 : 0), 0);
    this.ctx.restore();
}
Clock.prototype.setTime = function(date) {
    this.baseTime = date.getTime();
    this.baseDate = Date.now();
}
Clock.prototype.setSpeed = function(speed) {
    this.setTime(this.curDate());
    this.speed = speed;
}