using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    public Transform trans;
    List<myTriangle> triangleList=new List<myTriangle>();
    // Start is called before the first frame update
    void Start()
    {
        var objMesh=trans.GetComponent<MeshFilter>().mesh;
        for(int i=0;i<objMesh.triangles.Length;i+=3){
            myTriangle tempV=new myTriangle(objMesh.vertices[ objMesh.triangles[i]],objMesh.vertices[ objMesh.triangles[i+1]],objMesh.vertices[ objMesh.triangles[i+2]],objMesh.normals[objMesh.triangles[i]]);
            triangleList.Add(tempV);
        }
    }
bool IntersectTriangle(Vector3 orig,Vector3 dir,Vector3 v0, Vector3 v1, Vector3 v2)
{
    float t,u,v;
    // E1
    Vector3 E1 = v1 - v0;
    //E1=E1.normalized;
    // E2
    Vector3 E2 = v2 - v0;
    //E2=E2.normalized;
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
    //T=T.normalized;
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
        
    }
    trans.position=orig+t*dir;
    //trans.position=(1 - u - v) * v0 + u * v1 + v * v2;
    return true;
}
    // Update is called once per frame
    void Update()
    {   //int interscetNum=0;
        //for(int i=0;i<triangleList.Count;i++){
        //    
        //        
        //    Vector3 worldV1 = trans.TransformPoint(triangleList[i].p1);
        //    Vector3 worldV2 = trans.TransformPoint(triangleList[i].p2);
        //    Vector3 worldV3 = trans.TransformPoint(triangleList[i].p3);
        //   // Debug.LogError(worldV1+"worldV:"+worldVert);
        //    if(IntersectTriangle(transform.position,new Vector3(0,1,0),worldV1,worldV2,worldV3)){
        //        interscetNum++;
        //    }    
        //}
        //Debug.LogError("..."+interscetNum);
        //if(interscetNum%2==1){
        //    Debug.LogError(",,,"+interscetNum);
        //}
        if(IntersectTriangle(transform.position,new Vector3(0,1,0),new Vector3(0.5f,0,0.5f),new Vector3(-0.5f,0,0.5f),new Vector3(-0.5f,0,-0.5f)))
        Debug.LogError("xiangjiao");
        Debug.DrawRay(transform.position,new Vector3(0,1,0),Color.red);
    }
}
