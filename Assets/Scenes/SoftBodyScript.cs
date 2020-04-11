using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class myParticle{
        public Vector3 position;
        public Vector3 speedVector;
        public Vector3 gravityV;
        public Vector3 force;
        public Hashtable linkParticle=new Hashtable();
        public HashSet<int> sameParticle=new HashSet<int>();
        public myParticle(Vector3 p,Vector3 s,Vector3 f){
            this.position=p;this.speedVector=s;this.force=f;
        }
        public void show(){
            Debug.Log(this.position+","+this.speedVector+","+this.force+",");
            foreach(var key in linkParticle.Keys){
                Debug.Log("``````"+key+","+linkParticle[key]);
            }
        }
}
public class myTriangle{
    public Vector3 p1,p2,p3;//相对坐标
    public Vector3 normal;
    public myTriangle(Vector3 p_1,Vector3 p_2,Vector3 p_3,Vector3 n){
        p1=p_1;p2=p_2;p3=p_3;normal=n;
    }
}
public class SoftBodyScript : MonoBehaviour
{
    public float particleMass=1f;
    public float elasticityModulus=1f;
    public float Kd=0.1f;//阻尼系数
    public float n=0.01f;
    public float T=0.01f;
    public float R=0.01f;
    public float maxDist=0.1f;
    
    List<myParticle> particlesList=new List<myParticle>();
    int[] trianglesList;
    Vector3[] normalList;
    //MeshCollider mC;
    MeshFilter mF;
    Mesh originalMesh;
    Vector3[] softVerts;
    Vector3[] F;
    List<float> areaList=new List<float>();
    public bool gravityEnable=false;
    public float gravity;
    List<myTriangle> triangleList=new List<myTriangle>();
    public Transform ColliderObj;
    public float ContainR;
    HashSet<float> tSet=new HashSet<float>();
    // Start is called before the first frame update
    void Start()
    {   tSet.Clear();
        //mC=gameObject.GetComponent<MeshCollider>();
        mF = gameObject.GetComponent<MeshFilter> ();
        originalMesh=mF.mesh;
        trianglesList=mF.sharedMesh.triangles;
        normalList=mF.sharedMesh.normals;
        softVerts = mF.sharedMesh.vertices;
        foreach(Collider cld in GetComponents<Collider>()){
			Destroy(cld);
		}
		//mC = gameObject.AddComponent<MeshCollider> ();
		//mC.sharedMesh = originalMesh;
        //mC.convex = true; 
        
        for(int i=0;i<originalMesh.vertexCount;i++){
            myParticle temp=new myParticle(originalMesh.vertices[i],Vector3.zero,Vector3.zero);
            particlesList.Add(temp);
        }
        F=new Vector3[particlesList.Count];
        for(int i=0;i<originalMesh.triangles.Length;i+=3){
            if(!particlesList[originalMesh.triangles[i]].linkParticle.ContainsKey(originalMesh.triangles[i+1]))
                particlesList[originalMesh.triangles[i]].linkParticle.Add(originalMesh.triangles[i+1],(particlesList[originalMesh.triangles[i]].position-particlesList[originalMesh.triangles[i+1]].position).magnitude);
            if(!particlesList[originalMesh.triangles[i]].linkParticle.ContainsKey(originalMesh.triangles[i+2]))
                particlesList[originalMesh.triangles[i]].linkParticle.Add(originalMesh.triangles[i+2],(particlesList[originalMesh.triangles[i]].position-particlesList[originalMesh.triangles[i+2]].position).magnitude);
            if(!particlesList[originalMesh.triangles[i+1]].linkParticle.ContainsKey(originalMesh.triangles[i]))
                particlesList[originalMesh.triangles[i+1]].linkParticle.Add(originalMesh.triangles[i],(particlesList[originalMesh.triangles[i+1]].position-particlesList[originalMesh.triangles[i]].position).magnitude);
            if(!particlesList[originalMesh.triangles[i+1]].linkParticle.ContainsKey(originalMesh.triangles[i+2]))
                particlesList[originalMesh.triangles[i+1]].linkParticle.Add(originalMesh.triangles[i+2],(particlesList[originalMesh.triangles[i+1]].position-particlesList[originalMesh.triangles[i+2]].position).magnitude);
            if(!particlesList[originalMesh.triangles[i+2]].linkParticle.ContainsKey(originalMesh.triangles[i]))
                particlesList[originalMesh.triangles[i+2]].linkParticle.Add(originalMesh.triangles[i],(particlesList[originalMesh.triangles[i]].position-particlesList[originalMesh.triangles[i+2]].position).magnitude);
            if(!particlesList[originalMesh.triangles[i+2]].linkParticle.ContainsKey(originalMesh.triangles[i+1]))
                particlesList[originalMesh.triangles[i+2]].linkParticle.Add(originalMesh.triangles[i+1],(particlesList[originalMesh.triangles[i+2]].position-particlesList[originalMesh.triangles[i+1]].position).magnitude);
            //Debug.Log(originalMesh.triangles[i]);
        }
        for(int i=0;i<particlesList.Count;i++){
            for(int n=i+1;n<particlesList.Count;n++){
                if(particlesList[i].position==particlesList[n].position){
                    if(!particlesList[i].sameParticle.Contains(n))
                        particlesList[i].sameParticle.Add(n);
                    if(!particlesList[n].sameParticle.Contains(i))
                        particlesList[n].sameParticle.Add(i);
                }
            }
        }
        var objMesh=ColliderObj.GetComponent<MeshFilter>().mesh;
        for(int i=0;i<objMesh.triangles.Length;i+=3){
            myTriangle tempV=new myTriangle(objMesh.vertices[ objMesh.triangles[i]],objMesh.vertices[ objMesh.triangles[i+1]],objMesh.vertices[ objMesh.triangles[i+2]],objMesh.normals[objMesh.triangles[i]]);
            triangleList.Add(tempV);
        }
        
        //ContainR=ColliderObj.localScale.x>ColliderObj.localScale.y?ColliderObj.localScale.x:ColliderObj.localScale.y;
        //ContainR=ContainR>ColliderObj.localScale.z?ContainR:ColliderObj.localScale.z;
    }
    void OnDestroy(){
        //Debug.LogError("ondestroy");
        mF.sharedMesh=originalMesh;
        mF.sharedMesh.RecalculateBounds ();
        mF.sharedMesh.RecalculateNormals();
    }
    public float SignedAreaOfTriangle(Vector3 pt0, Vector3 pt1, Vector3 pt2)
    {
        pt0 = new Vector3(pt0.x * transform.localScale.x, pt0.y * transform.localScale.y, pt0.z * transform.localScale.z);
        pt1 = new Vector3(pt1.x * transform.localScale.x, pt1.y * transform.localScale.y, pt1.z * transform.localScale.z);
        pt2 = new Vector3(pt2.x * transform.localScale.x, pt2.y * transform.localScale.y, pt2.z * transform.localScale.z);
        float a = (pt1 - pt0).magnitude;
        float b = (pt2 - pt1).magnitude;
        float c = (pt0 - pt2).magnitude;
        float p = (a + b + c) * 0.5f;
        return Mathf.Sqrt(p * (p - a) * (p - b) * (p - c));
    }
    public float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3) {
        var v321 = p3.x*p2.y*p1.z;
        var v231 = p2.x*p3.y*p1.z;
        var v312 = p3.x*p1.y*p2.z;
        var v132 = p1.x*p3.y*p2.z;
        var v213 = p2.x*p1.y*p3.z;
        var v123 = p1.x*p2.y*p3.z;
        return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
    }
    //计算体积
    public float VolumeOfMesh() {
        float v=0;
        for(int i=0;i<trianglesList.Length;i+=3){
            v+=SignedVolumeOfTriangle(particlesList[trianglesList[i]].position,particlesList[trianglesList[i+1]].position,particlesList[trianglesList[i+2]].position);
        }
        return v;
    }
    //void OnCollisionEnter(Collision col) {
	//	Debug.LogError(col.gameObject.name);
    //    ContactPoint[] contcts = col.contacts;
    //    foreach (ContactPoint contct in contcts) {
	//		Vector3 lP = contct.point;
	//		for(int i = 0; i < particlesList.Count; i++){
	//			Vector3 worldVert = transform.TransformPoint(particlesList[i].position);
	//			float tmpDist = Vector3.Distance(lP, worldVert);
	//			if(tmpDist < maxDist){
    //                //worldVert += contct.normal*0.1f;
	//				particlesList[i].speedVector=particlesList[i].speedVector.magnitude*contct.normal*0.0f;
	//				//particlesList[i].position = transform.InverseTransformPoint(worldVert);
	//			}
	//		}
	//	}
	//}
    //void OnCollisionStay(Collision col) {
	//	Debug.LogError(col.gameObject.name);
    //    ContactPoint[] contcts = col.contacts;
    //    foreach (ContactPoint contct in contcts) {
	//		Vector3 lP = contct.point;
	//		for(int i = 0; i < particlesList.Count; i++){
	//			Vector3 worldVert = transform.TransformPoint(particlesList[i].position);
	//			float tmpDist = Vector3.Distance(lP, worldVert);
	//			if(tmpDist < maxDist){
    //                //worldVert += contct.normal*0.1f;
	//				particlesList[i].speedVector=particlesList[i].speedVector.magnitude*contct.normal*0.0f;
	//				//particlesList[i].position = transform.InverseTransformPoint(worldVert);
	//			}
	//		}
	//	}
	//}
    // Update is called once per frame
    // Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool IntersectTriangle(Vector3 orig,Vector3 dir,Vector3 v0, Vector3 v1, Vector3 v2)
{
    float t,u,v;
    // E1
    Vector3 E1 = v1 - v0;

    // E2
    Vector3 E2 = v2 - v0;

    // P
    Vector3 P =Vector3.Cross(dir,E2);
    //Vector3 P = dir.Cross(E2);

    // determinant
    //float det = E1.Dot(P);
    float det =Vector3.Dot(E1,P);
    // keep det > 0, modify T accordingly
    Vector3 T;
    if( det >0 )
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if( det < 0.0001f )
        return false;

    // Calculate u and make sure u <= 1
    u = Vector3.Dot(T,P);//T.Dot(P);
    if( u < 0.0f || u > det )
        return false;

    // Q
    //Vector3 Q = T.Cross(E1);
    Vector3 Q = Vector3.Cross(T,E1);
    // Calculate v and make sure u + v <= 1
    v =Vector3.Dot(dir,Q);// dir.Dot(Q);
    if( v < 0.0f || u + v > det )
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t =Vector3.Dot(E2,Q);// E2.Dot(Q);

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;
    if(t<0){
        return false;
    }//一个方向
    
    //在边上
    if(u==0||v==0||v+u==1){
        if(tSet.Contains(t)){
            return false;
        }
        else{
            tSet.Add(t);
        }
    }
    return true;
}
//返回沿三角面法线与其相交的t值，若不相交，返回-1
float IntersectTriangleReturn_T(Vector3 orig,Vector3 dir,Vector3 v0, Vector3 v1, Vector3 v2)
{
    float t,u,v;
    // E1
    Vector3 E1 = v1 - v0;

    // E2
    Vector3 E2 = v2 - v0;

    // P
    Vector3 P =Vector3.Cross(dir,E2);
    //Vector3 P = dir.Cross(E2);

    // determinant
    //float det = E1.Dot(P);
    float det =Vector3.Dot(E1,P);
    // keep det > 0, modify T accordingly
    Vector3 T;
    if( det >0 )
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if( det < 0.0001f )
        return -1.0f;

    // Calculate u and make sure u <= 1
    u = Vector3.Dot(T,P);//T.Dot(P);
    if( u < 0.0f || u > det )
        return -1.0f;

    // Q
    //Vector3 Q = T.Cross(E1);
    Vector3 Q = Vector3.Cross(T,E1);
    // Calculate v and make sure u + v <= 1
    v =Vector3.Dot(dir,Q);// dir.Dot(Q);
    if( v < 0.0f || u + v > det )
        return -1.0f;

    // Calculate t, scale parameters, ray intersects triangle
    t =Vector3.Dot(E2,Q);// E2.Dot(Q);

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;
    if(t<0){
        return -1.0f;
    }//一个方向
    
    
    return t;
}
    void Update()
    {
        /*
            1 Loop over all particles:
[1.1] Calculate and accumulate gravity and spring forces
for all particles.
2 Calculate volume of the soft body
3 Loop over all faces:
[3.1] Calculate pressure force acting on the face
[3.1.1] Calculate field of the face
[3.1.2] Calculate the pressure value
[3.1.3] Loop over particles which define the face:
[3.1.3.] Multiply result by field of the face and ˆn
[3.1.3.2] Accumulate finally pressure force to the particle.
4 Integrate momentum equation
5 Resolve collision handling and response.
6 Move particles
        */
        
        tSet.Clear();
        for(int i=0;i<particlesList.Count;i++){
            Vector3 tempF=Vector3.zero;
            //tempF= new Vector3(0,-0.098f*particleMass,0);
            foreach(var linkKey in particlesList[i].linkParticle.Keys){   
                tempF+=elasticityModulus*(particlesList[(int)linkKey].position- particlesList[i].position).normalized*((particlesList[(int)linkKey].position- particlesList[i].position).magnitude-(float)particlesList[i].linkParticle[linkKey])
                //+Kd*Vector3.Dot((particlesList[i].speedVector-particlesList[(int)linkKey].speedVector),particlesList[(int)linkKey].position- particlesList[i].position.normalized)*(particlesList[(int)linkKey].position- particlesList[i].position).normalized
                ;
            }
            //弹簧拉力
            F[i]=tempF;
            
        }
        float volume=VolumeOfMesh()*transform.localScale.x*transform.localScale.y*transform.localScale.z;
        float presure=n*T*R/volume;
        Debug.Log(volume);
        areaList.Clear();
        for(int i=0;i<trianglesList.Length;i+=3){
            float tempArea=SignedAreaOfTriangle(particlesList[trianglesList[i]].position,particlesList[trianglesList[i+1]].position,particlesList[trianglesList[i+2]].position);
            areaList.Add(tempArea);
            F[trianglesList[i]]+= normalList[trianglesList[i]]*tempArea*presure;
            F[trianglesList[i+1]]+= normalList[trianglesList[i+1]]*tempArea*presure;
            F[trianglesList[i+2]]+= normalList[trianglesList[i+2]]*tempArea*presure;
        }
        //将相同点的收到的压力，拉力合并
        for(int i=0;i<particlesList.Count;i++){
            Vector3 tempF=F[i];
            foreach(var sp in particlesList[i].sameParticle){
                tempF+=F[sp];
            }
            particlesList[i].force=tempF;
            
            
            //Debug.LogError("force   "+i+":"+particlesList[i].force.magnitude);
            
            particlesList[i].force+=-(particlesList[i].speedVector-particlesList[i].gravityV).normalized*Kd;
            if(gravityEnable){
                particlesList[i].force+=new Vector3(0,gravity*particleMass,0);
                particlesList[i].gravityV+=new Vector3(0,gravity,0);
            }
            //particlesList[i].force+=new Vector3(0,-0.098f*particleMass,0);
            particlesList[i].speedVector+=particlesList[i].force/particleMass;
            particlesList[i].position+=particlesList[i].speedVector;
            //Vector3 worldVert = transform.TransformPoint(particlesList[i].position);
            //if(worldVert.y<0f){
            //    worldVert.y=0f;
            //    particlesList[i].position= transform.InverseTransformPoint(worldVert);
            //}
            Vector3 worldVert = transform.TransformPoint(particlesList[i].position);
            if(Vector3.Distance(worldVert,ColliderObj.position)<ContainR){
                int interscetNum=0;
                for(int n=0;n<triangleList.Count;n++){
                    Vector3 worldV1 = ColliderObj.TransformPoint(triangleList[n].p1);
                    Vector3 worldV2 = ColliderObj.TransformPoint(triangleList[n].p2);
                    Vector3 worldV3 = ColliderObj.TransformPoint(triangleList[n].p3);
                   // Debug.LogError(worldV1+"worldV:"+worldVert);
                    if(IntersectTriangle(worldVert,particlesList[i].speedVector.normalized,worldV1,worldV2,worldV3)){
                        interscetNum++;
                    }
                }
                //Debug.LogError(i+"..."+interscetNum);
                if(interscetNum%2==1){
                   //Debug.LogError(i+",,,"+interscetNum);
                    //particlesList[i].position-=particlesList[i].speedVector;
                    //particlesList[i].speedVector=Vector3.zero;
                    //获取最近的obj三角面i和u，v。将position=最近的碰撞点
                    int tempTriangle=0;
                    float tempT=float.MaxValue;
                    for(int n=0;n<triangleList.Count;n++){
                        Vector3 worldV1 = ColliderObj.TransformPoint(triangleList[n].p1);
                        Vector3 worldV2 = ColliderObj.TransformPoint(triangleList[n].p2);
                        Vector3 worldV3 = ColliderObj.TransformPoint(triangleList[n].p3);
                        var tempvalue =IntersectTriangleReturn_T(worldVert,triangleList[n].normal,worldV1,worldV2,worldV3);
                        if(tempvalue!=-1){
                            if(tempT>tempvalue){
                                tempT=tempvalue;
                                tempTriangle=n;
                            }
                        }
                        
                    }
                    particlesList[i].position=transform.InverseTransformPoint(worldVert
                    +tempT*triangleList[tempTriangle].normal
                    );

                    particlesList[i].speedVector=particlesList[i].speedVector-triangleList[tempTriangle].normal*Vector3.Dot(triangleList[tempTriangle].normal,particlesList[i].speedVector);
                    particlesList[i].gravityV=new Vector3(0,particlesList[i].speedVector.y,0);
                    //Debug.LogError(tempTriangle+"temp:"+triangleList[tempTriangle].normal);
                }
            }
            
            softVerts[i]=particlesList[i].position;
        }
        mF.sharedMesh.vertices = softVerts;
		mF.sharedMesh.RecalculateBounds ();
        mF.sharedMesh.RecalculateNormals();
        //mC.sharedMesh=mF.sharedMesh;

    }
}
