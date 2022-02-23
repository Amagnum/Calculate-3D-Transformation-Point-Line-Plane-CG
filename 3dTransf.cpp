#include <bits/stdc++.h>
#include <Eigen>

/*
g++ -I ./lib/eigen-3.3.9/Eigen 3dTransf.cpp -o e3ds
./e3ds < 3d.in
*/

using namespace Eigen;
using namespace std;


double sround(double value, int prec){
	return round( value * pow(10,prec)) / pow(10,prec);
}

MatrixXd mround(MatrixXd mat, int prec){
	for (int i = 0; i < mat.rows(); ++i)
	{
		for (int j = 0; j < mat.cols(); ++j)
		{
			mat(i,j)=sround(mat(i,j), prec);
		}
	}
	return mat;
}

MatrixXd Shx(double sx, double sy, double sz, MatrixXd in, int rnd){
	Matrix4d tm;
	tm << 1 , 0, 0, 0,
		  sy , 1, 0, 0,
		  sz , 0, 1, 0,
		  0 , 0, 0, 1;
	tm = mround(tm,rnd);
	cout << "\nApplying Shear X:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Shy(double sx, double sy, double sz, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << 1 , sx, 0, 0,
		  0 , 1, 0, 0,
		  0 , sz, 1, 0,
		  0 , 0, 0, 1;
	tm = mround(tm,rnd);
	cout << "\nApplying Shear Y:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Shz(double sx, double sy, double sz, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << 1 , 0, sx, 0,
		  0 , 1, sy, 0,
		  0 , 0, 1, 0,
		  0 , 0, 0, 1;
	tm = mround(tm,rnd);
	cout << "\nApplying Shear Z:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Rx(double th, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << 1 , 0, 0, 0,
		  0 , cos(th), -sin(th), 0,
		  0 , sin(th), cos(th), 0,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Rx:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Ry(double th, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << cos(th) , 0, sin(th), 0,
		  0 , 1, 0, 0,
		  -sin(th), 0,cos(th), 0,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Ry:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Rz(double th, MatrixXd in, int rnd){
	MatrixXd tm(4,4);
	tm << 
		  cos(th), -sin(th), 0, 0,
		  sin(th), cos(th), 0, 0,
		  0 , 0, 1, 0,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Rz:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Trans(double tx, double ty, double tz, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << 1 , 0, 0, tx,
		  0 , 1, 0, ty,
		  0 , 0, 1, tz,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Translation:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Scale(double sx, double sy, double sz, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << sx , 0, 0, 0,
		  0 , sy, 0, 0,
		  0 , 0, sz, 0,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Scale:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

MatrixXd Proj(double Q,Vector3d plane, Vector3d dir, MatrixXd in,  int rnd){
	 Matrix4d Mgen;

    // Q is distance to cop.
	double dx=dir(0),dy=dir(1),dz=dir(2), zp = plane(2);

    Mgen << 1, 0, -dx/dz,     zp*dx/dz,
    		0, 1, -dy/dz,     zp*dy/dz,
    		0, 0, -zp/(Q*dz), (zp*zp/(Q*dz))+zp,
    		0, 0, -1/(Q*dz),  zp/(Q*dz)+1;

	Mgen = mround(Mgen,rnd);
	cout << "\nApplying Plane Projection (Remember to / by W):\n"<<Mgen<<"\n.........."<<endl;
    return Mgen*in;
}

MatrixXd PerCVVtoParCVV(double zmin, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << 1 , 0, 0, 0,
		  0 , 1, 0, 0,
		  0 , 0, 1/(1+zmin), -zmin/(1+zmin),
		  0 , 0, -1, 0;

	tm = mround(tm,rnd);
	cout << "\nApplying Scale:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}

// #############################################################

MatrixXd Refl(bool x, bool y, bool z, MatrixXd in,  int rnd){
	Matrix4d tm;
	tm << (y&z?-1:1) , 0, 0, 0,
		  0 , (x&z?-1:1), 0, 0,
		  0 , 0, (x&y?-1:1), 0,
		  0 , 0, 0, 1;

	tm = mround(tm,rnd);
	cout << "\nApplying Reflection:\n"<<tm<<"\n.........."<<endl;
	return tm*in;
}


MatrixXd ReflAP2(double cx, double cy, double cz, MatrixXd in,  int rnd){
	Matrix4d Reff;
	
	Vector3d vec;
	vec << cx , cy , cz;
	vec.normalize();
	cx = vec(0);
	cy = vec(1);
	cz = vec(2);

	Reff << 1 - 2*cx*cx, -2*cy*cx, -2*cz*cx, 0,
		   -2*cy*cx , 1 - 2*cy*cy, -2*cy*cz, 0,
		   -2*cz*cx , -2*cy*cz, 1 - 2*cz*cz, 0,
		   0 , 0, 0, 1;
	Reff = mround(Reff,rnd);
	cout << "\nApplying Direct ReflAP2:\n"<<Reff<<"\n.........."<<endl;
	
	return Reff*in;
}

MatrixXd RaA2(double th, double cx, double cy, double cz, MatrixXd in, int rnd){
	
	double ct = cos(th); 

	Vector3d vec;
	vec << cx , cy , cz;
	vec.normalize();
	cx = vec(0);
	cy = vec(1);
	cz = vec(2);

	Matrix4d Rot;
	
	Rot << ct + cx*cx*(1-ct) , cx*cy*(1-ct)-cz*sin(th), cx*cz*(1-ct)+cy*sin(th), 0,
		  cx*cy*(1-ct)+cz*sin(th) , ct + cy*cy*(1-ct), cz*cy*(1-ct)-cx*sin(th), 0,
		  cx*cz*(1-ct)-cy*sin(th) , cz*cy*(1-ct)+cx*sin(th), ct + cz*cz*(1-ct), 0,
		  						0 , 				0, 						 0, 1;
	Rot = mround(Rot,rnd);
	cout << "\nApplying Direct RaA2:\n"<<Rot<<"\n.........."<<endl;

	return Rot*in;
}

MatrixXd ReflAP(double a, double b, double c, double d, MatrixXd in,  int rnd){
	MatrixXd ans;
	double th = acos(b/sqrt(pow(a,2)+pow(c,2)+pow(b,2)));
	
	ans = Trans(0,-d/b,0,in,rnd);
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;

	ans = Rz(-th,ans,rnd);
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;

	ans = Refl(1,0,1,ans,rnd);
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;

	ans = Rz(th,ans,rnd);
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;

	ans = Trans(0,d/b,0,ans,rnd);
	ans = mround(ans,rnd);
	
	return ans;
}


MatrixXd RaA(double th, double a, double b, double c, MatrixXd in, int rnd){
	double d = sqrt(pow(b,2)+pow(c,2));
	double l = sqrt(pow(a,2)+pow(b,2)+pow(c,2));

	cout << "\nApplying RaA:\n.........."<<endl;
	cout<< a <<" "<< b<<" "<< c<<endl;

	Matrix4d Rx,Rxi,Ry,Ryi;
	MatrixXd ans = in;
	
	Rx << 1 , 0, 0, 0,
		  0 , c/d, -b/d, 0,
		  0 , b/d, c/d, 0,
		  0 , 0, 0, 1;
    ans = Rx*ans;
    cout << "\nApplying spec Rx:\n"<<Rx<<"\n.........."<<endl;
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;


	Ry << d/l , 0, -a/l, 0,
		  0 , 1, 0, 0,
		  a/l , 0, d/l, 0,
		  0 , 0, 0, 1;
	ans = Ry*ans;
	cout << "\nApplying spec Ry:\n"<<Ry<<"\n.........."<<endl;
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;


	ans = Rz(th,ans,rnd);
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;


	Ryi << d/l , 0, a/l, 0,
		  0 , 1, 0, 0,
		  -a/l , 0, d/l, 0,
		  0 , 0, 0, 1;
	ans = Ryi*ans;
	cout << "\nApplying spec Ry inv:\n"<<Ryi<<"\n.........."<<endl;
	ans = mround(ans,rnd);
	cout<<ans<<"\n.........."<<endl;


	Rxi << 1 , 0, 0, 0,
		  0 , c/d, b/d, 0,
		  0 , -b/d, c/d, 0,
		  0 , 0, 0, 1;
	ans = Rxi*ans;
	cout << "\nApplying spec Rx inv:\n"<<Rxi<<"\n.........."<<endl;
	ans = mround(ans,rnd);
	
	return ans;
}


MatrixXd Npar(double Umin,double Umax, double Vmin,double Vmax, double front, double back,
 Vector3d vup, Vector3d vpn,Vector3d prp,Vector3d vrp , MatrixXd points, int rnd){
	
	cout << "Applying Npar:\n.........."<<endl;

	double mid_x = -(Umin + Umax) * 0.5;
	double mid_y = -(Vmax + Vmin) * 0.5;

	points = Trans(-vrp(0),-vrp(1),- vrp(2), points, rnd);
	points = mround(points,rnd);
	cout <<"\n"<< points<<"\n.........."<<endl;

	// Rotation 
	Vector3d r_z = vpn.normalized();
	Vector3d r_x = vup.cross(r_z);
	r_x = r_x.normalized();
	Vector3d r_y = r_z.cross(r_x);
	Matrix4d rot;

	rot <<  (r_x(0)), (r_x(1)), (r_x(2)), 0,
			(r_y(0)), (r_y(1)), (r_y(2)), 0,
			(r_z(0)), (r_z(1)), (r_z(2)), 0,
				 0,		 0,		 0, 1;
	rot = mround(rot,rnd);
	points = rot * points;
	
	cout << "Applying spec Rot:\n"<<rot<<"\n.........."<<endl;
	points = mround(points,rnd);
	cout <<"\n"<< points <<"\n.........."<<endl;


	// Sheering
	Vector3d cw;
	cw(0) = -1* mid_x;
	cw(1) = -1* mid_y;
	cw(2) = 0;
	Vector3d dop = cw - prp;
	double shx_par = - dop(0)/dop(2);
	double shy_par = - dop(1)/dop(2);
	
	Matrix4d Shpar;
	Shpar << 1, 0, shx_par, 0,
			 0, 1, shy_par, 0,
			 0, 0,       1, 0,
			 0, 0,		 0, 1;

    points = Shpar * points;
	points = mround(points,rnd);
    cout << "Applying Spec Sheering:\n"<<Shpar<<"\n.........."<<endl;
	cout <<"\n"<<points <<"\n.........."<<endl;
    

	// Translation
	points = Trans( mid_x, mid_y, -front, points, rnd);
	points = mround(points,rnd);
	cout <<"\n"<< points<<"\n.........."<<endl;
    

	// Scale
	points = Scale(2/(Umax-Umin),2/(Vmax-Vmin),1/(front - back), points, rnd);
	points = mround(points,rnd);
	cout <<"\n"<< points<<"\n.........."<<endl;
    

	return points;
}


int main(){
	
	int np=1;
	int rnd=9;
	string dummy;
		
	cout<<"Enter no of Points\n";
	cin >> np;

	MatrixXd points(4,np);

	for(int j=0; j<4; j++){
		for(int i=0; i<np; i++){
			double temp=0;
			cin>>temp;
			points(j,i) = temp;
		}
	}

	cout<<points<<endl;

	cin >> dummy;
	assert(dummy=="rnd");
	cin >> rnd;
		
	int mods;
	cin>>mods;

	MatrixXd ans = points;

	while(mods--){
		string command;
		cin >> command;
			if(command== "Rx"){
				double xDegrees = 90;
				cin >> xDegrees;
				double x = xDegrees*3.14159/180;
				ans = Rx(x,ans, rnd);
				
			}
			else if(command== "Ry"){
				double xDegrees = 90;
				cin >> xDegrees;
				double x = xDegrees*3.14159/180;
				ans = Ry(x,ans, rnd);
				
			}
			else if(command== "Rz"){
				double xDegrees = 90;
				cin >> xDegrees;
				double x = xDegrees*3.14159/180;
				ans = Rz(x,ans, rnd);
				
			}
			else if(command== "RaA"){
				double xDegrees = 90;
				cin >> xDegrees;
				double x = xDegrees*3.14159/180;

				double xi, xj, xk; // axis direnction vector
				cin >> xi >> xj >> xk;
				ans = RaA(x,xi,xj,xk, ans, rnd);
			}
			else if(command== "RaA2"){
				double xDegrees = 90;
				cin >> xDegrees;
				double x = xDegrees*3.14159/180;

				double xi, xj, xk; // axis direnction vector (Normalised)
				cin >> xi >> xj >> xk;
				ans = RaA2(x,xi,xj,xk, ans, rnd);
			}
			else if(command== "Shx"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Shx(sx,sy,sz,ans, rnd);
				
			}
			else if(command== "Shy"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Shy(sx,sy,sz,ans, rnd);
				
			}
			else if(command== "Shz"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Shz(sx,sy,sz,ans, rnd);
				
			}
			else if(command== "Scale"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Scale(sx,sy,sz,ans, rnd);
				
			}
			else if(command== "Trans"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Trans(sx,sy,sz,ans, rnd);
				
			}
			else if(command== "Proj"){
				double Q,x,y,z;
				Vector3d plane,dir;
				cin >> Q;
				cin >> dummy >> x >> y >> z;
				assert(dummy=="plane");
				plane << x , y , z;


				cin >> dummy >> x >> y >> z;
				assert(dummy=="dir");
				dir << x , y , z;

				ans = Proj(Q, plane, dir, ans, rnd);

			}
			else if(command== "Refl"){
				double sx,sy,sz;
				cin>>sx>>sy>>sz;
				ans = Refl(sx,sy,sz,ans, rnd);
				
			}else if(command== "ReflAP2"){
				double a,b,c;
				cin >> a >> b >> c;
				ans = ReflAP2(a,b,c,ans, rnd);

			}
			else if(command== "ReflAP"){
				double a,b,c,d;
				cin >> a >> b >> c >> d;
				ans = ReflAP(a,b,c,d,ans, rnd);
				
			}
			else if(command== "PerCVVtoParCVV"){
				//PerCVVtoParCVV
				double zmin;
				cin >> zmin;
				ans = PerCVVtoParCVV(zmin,ans, rnd);
			}
			else if(command== "Npar"){
				double Umin, Umax, Vmin, Vmax, front, back, x,y,z;
				Vector3d vup, vpn, prp, vrp;
				string temp;

				cin >> Umin >> Umax >> Vmin >> Vmax;
				cin >> front >> back;
				
				cin >> temp >> x >> y >> z;
				assert(temp=="vrp");
				vrp(0) = x;
				vrp(1) = y;
				vrp(2) = z;

				cin >> temp >> x >> y >> z;
				assert(temp=="vpn");
				vpn(0) = x;
				vpn(1) = y;
				vpn(2) = z;

				cin >> temp >> x >> y >> z;
				assert(temp=="vup");
				vup(0) = x;
				vup(1) = y;
				vup(2) = z;

				cin >> temp >> x >> y >> z;
				assert(temp=="prp");
				prp(0) = x;
				prp(1) = y;
				prp(2) = z;
				
				ans = Npar(Umin, Umax, Vmin, Vmax, front, back, vup, vpn, prp ,vrp, points, rnd);
			}
			else{
				
				cout<< "ERR: command not identified\n";
				break;
			}
			cout<<ans<<"\n........ END "<<command<<" ........\n\n";
	}

	ans = mround(ans,rnd);
	cout<<"\nFinal Ans:\n"<<ans<<endl;

	return 0;
}

/*
A (0, 3, 1), B(3, 3, 2), C(3, 0, 0), D(0, 0, 0)
*/