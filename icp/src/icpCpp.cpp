/*=================================================================
 *
 * ICP algorithm implemented in c++
 *
 * written by
 *
 * Per Bergstr�m 2007-10-09
 *
 * email: per.bergstrom@ltu.se
 *
 * Uses kd-tree written by Guy Shechter
 * http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=4586&objectType=file
 *
 *=================================================================*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include "icp/kdtree_common.h"

namespace icp {

/* Input Arguments */

//#define	model	prhs[0]
//#define	data	prhs[1]
//#define	weights	prhs[2]
//#define	randvec	prhs[3]
//#define	sizerand	prhs[4]
//#define	treeptr	prhs[5]
//#define	iter	prhs[6]


/* Output Arguments */

#define	TR	plhs[0]
#define	TT     plhs[1]

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

double pwr2(double a){
    return a*a;
}

void icp(
double *trpr,
double *ttpr,
double *modelz,
unsigned int nmodelz,
double *dataz,
double *qltyz,
unsigned int ndataz,
unsigned int *randvecz,
unsigned int nrandvecz,
unsigned int nrandz,
unsigned int iimax,
Tree *pointer_to_tree
)
{
    
    unsigned int i,itemp,j,k,ii,cou=0;
    double R[9],T[3],distcc;
    double datam[3],modelm[3],MM[3],Spx[9],quasum;
    unsigned short int bol=1;
    double SIGMA[3];
    double SpxTSpx[6];
    double A,B,C;
    double sqrtA23B;
    double x0,f0;
    double SIp,difl;
    double invmat[6];
    double V[9];
    double U[9];
    Tree *tree;
    
    tree = pointer_to_tree;
    
    trpr[0]=1.0;
    trpr[4]=1.0;
    trpr[8]=1.0;
    
    if((nrandz+1)>ndataz){
        bol=0;
    }
    
    for(ii=0;ii<iimax;ii++){
        
        for(i=0;i<9;i++){
            Spx[i]=0.0;
        }
        for(i=0;i<3;i++){
            modelm[i]=0.0;
            datam[i]=0.0;
        }
        
        quasum=0.0;
        for(itemp=0;itemp<ndataz;itemp++){
            
            if(bol){
                if(itemp<nrandz){
                    i=randvecz[cou];
                    
                    if(cou<(nrandvecz-1)){
                        cou++;
                    }
                    else{
                        cou=0;
                    }
                }
                else{
                    break;
                }
            }
            else{
                i=itemp;
            }
            
            if(qltyz[i]>0.0){
                
                T[0]=trpr[0]*dataz[3*i]+trpr[3]*dataz[3*i+1]+trpr[6]*dataz[3*i+2]+ttpr[0];
                T[1]=trpr[1]*dataz[3*i]+trpr[4]*dataz[3*i+1]+trpr[7]*dataz[3*i+2]+ttpr[1];
                T[2]=trpr[2]*dataz[3*i]+trpr[5]*dataz[3*i+1]+trpr[8]*dataz[3*i+2]+ttpr[2];
                
                distcc=0.0;
                A=0.0;
                
                run_queries(tree->rootptr,T,1,3,&A,&distcc, RETURN_INDEX);
                
                k=(unsigned int)A;
                
                MM[0]=qltyz[i]*modelz[3*k];
                MM[1]=qltyz[i]*modelz[3*k+1];
                MM[2]=qltyz[i]*modelz[3*k+2];
                
                datam[0]+=qltyz[i]*T[0];
                datam[1]+=qltyz[i]*T[1];
                datam[2]+=qltyz[i]*T[2];
                
                modelm[0]+=MM[0];
                modelm[1]+=MM[1];
                modelm[2]+=MM[2];
                
                for(j=0;j<3;j++){
                    Spx[j]+= MM[j]*T[0];
                    Spx[j+3]+= MM[j]*T[1];
                    Spx[j+6]+= MM[j]*T[2];
                }
                quasum+=qltyz[i];
            }
            
        }
        
        if(quasum<1e-12){
            break;
        }
        
        for(j=0;j<3;j++){
            modelm[j]=modelm[j]/quasum;
            datam[j]=datam[j]/quasum;
        }
        
        for(j=0;j<3;j++){
            Spx[j]-=quasum*(modelm[j]*datam[0]);
            Spx[j+3]-=quasum*(modelm[j]*datam[1]);
            Spx[j+6]-=quasum*(modelm[j]*datam[2]);
        }
        
        k=1;
        if(ii<1){
            
            distcc=0.0;
            for(j=0;j<9;j++){
                distcc+=Spx[j]*Spx[j];
            }
            
            distcc=distcc/pwr2(quasum);
            
            if(distcc<1e-3){
                k=0;
//                T[0]=modelm[0]-datam[0];
//                T[1]=modelm[1]-datam[1];
//                T[2]=modelm[2]-datam[2];
                T[0] = modelz[0] - dataz[0];
                T[1] = modelz[1] - dataz[1];
                T[2] = modelz[2] - dataz[2];
                R[0]=1.0;R[3]=0.0;R[6]=0.0;
                R[1]=0.0;R[4]=1.0;R[7]=0.0;
                R[2]=0.0;R[5]=0.0;R[8]=1.0;
            }
            
        }
        
        if(k) {
            
    /* get R*/
            
            SpxTSpx[0]=Spx[0]*Spx[0]+Spx[1]*Spx[1]+Spx[2]*Spx[2];
            SpxTSpx[1]=Spx[3]*Spx[3]+Spx[4]*Spx[4]+Spx[5]*Spx[5];
            SpxTSpx[2]=Spx[6]*Spx[6]+Spx[7]*Spx[7]+Spx[8]*Spx[8];
            SpxTSpx[3]=Spx[0]*Spx[3]+Spx[1]*Spx[4]+Spx[5]*Spx[2];
            SpxTSpx[4]=Spx[3]*Spx[6]+Spx[4]*Spx[7]+Spx[5]*Spx[8];
            SpxTSpx[5]=Spx[0]*Spx[6]+Spx[1]*Spx[7]+Spx[2]*Spx[8];
            
            
    /*   CharacteristicPolynomial  sigma^3-A*sigma^2-B*sigma+C   */
            
            A=SpxTSpx[2]+SpxTSpx[1]+SpxTSpx[0];
            B=SpxTSpx[5]*SpxTSpx[5]+SpxTSpx[4]*SpxTSpx[4]-SpxTSpx[2]*SpxTSpx[1]-SpxTSpx[2]*SpxTSpx[0]+SpxTSpx[3]*SpxTSpx[3]-SpxTSpx[1]*SpxTSpx[0];
            C=-2*SpxTSpx[5]*SpxTSpx[3]*SpxTSpx[4]+SpxTSpx[5]*SpxTSpx[5]*SpxTSpx[1]+SpxTSpx[4]*SpxTSpx[4]*SpxTSpx[0]+SpxTSpx[2]*SpxTSpx[3]*SpxTSpx[3]-SpxTSpx[2]*SpxTSpx[1]*SpxTSpx[0];
            
            sqrtA23B=sqrt(A*A+3*B);
            
            x0=(A-sqrtA23B)/3.0;
            f0=(x0*x0-A*x0-B)*x0;
            
            SIGMA[2]=MIN(MAX((x0*(C+2*f0-2*sqrt(f0*(f0+C)))/f0),0),0.5*x0);
            
            x0=(A+sqrtA23B)/3.0;
            f0=x0*x0*x0-A*x0*x0-B*x0+C;
            
            SIGMA[0]=MAX(MIN((x0*(B*A-C)+2*x0*f0+2*(x0-A)*sqrt(f0*(f0+B*A-C))-A*f0)/(f0+B*A-C),A),0.5*(A+x0));
            
            for(k=0;k<3;k++){
                
                if(k==0){
                    j=0;
                }
                else if(k==1){
                    j=2;
                }
                else if(k==2){
                    j=1;
                    SIGMA[1]=A-SIGMA[0]-SIGMA[2];
                }
                
        /* Newton-Raphson */
                
                for(i=0;i<50;i++){
                    SIp=SIGMA[j];
                    difl=(-3*SIGMA[j]+2*A)*SIGMA[j]+B;
                    if(fabs(difl)>1e-15){
                        SIGMA[j]=((-2*SIGMA[j]+A)*SIGMA[j]*SIGMA[j]+C)/difl;
                        if(fabs(SIGMA[j]-SIp)<1e-25){
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }
            }
            
            k=0;
            if(fabs(SIGMA[1]-SIGMA[0])<1e-12){
                k=1;
            }
            
    /* eigenvalues found, corresponding eigenvectors V[i] ... */
            
            for(i=0;i<3;i++){
                
                invmat[0]=SpxTSpx[2]*SpxTSpx[1]-SpxTSpx[1]*SIGMA[i]-SIGMA[i]*SpxTSpx[2]+SIGMA[i]*SIGMA[i]-SpxTSpx[4]*SpxTSpx[4];
                invmat[1]=SpxTSpx[4]*SpxTSpx[5]-SpxTSpx[3]*SpxTSpx[2]+SpxTSpx[3]*SIGMA[i];
                invmat[2]=SpxTSpx[3]*SpxTSpx[4]-SpxTSpx[5]*SpxTSpx[1]+SpxTSpx[5]*SIGMA[i];
                invmat[3]=SpxTSpx[2]*SpxTSpx[0]-SpxTSpx[0]*SIGMA[i]-SIGMA[i]*SpxTSpx[2]+SIGMA[i]*SIGMA[i]-SpxTSpx[5]*SpxTSpx[5];
                invmat[4]=-SpxTSpx[4]*SpxTSpx[0]+SpxTSpx[4]*SIGMA[i]+SpxTSpx[3]*SpxTSpx[5];
                invmat[5]=SpxTSpx[1]*SpxTSpx[0]-SpxTSpx[0]*SIGMA[i]-SpxTSpx[1]*SIGMA[i]+SIGMA[i]*SIGMA[i]-SpxTSpx[3]*SpxTSpx[3];
                
                if(i<2){
                    V[3*i]=invmat[0];
                    V[3*i+1]=invmat[1];
                    V[3*i+2]=invmat[2];
                    
                    if(k){
                        if(i==1){
                            V[3]=invmat[1];
                            V[4]=invmat[3];
                            V[5]=invmat[4];
                            
                            distcc=V[3]*V[0]+V[4]*V[1]+V[5]*V[2];
                            V[3]=V[3]-distcc*V[0];
                            V[4]=V[4]-distcc*V[1];
                            V[5]=V[5]-distcc*V[2];
                        }
                    }
                    
                }
                else {
                    
                    /* Eigen vectors corresponding to symmetric positiv definite matrices
                     are orthogonal. */
                    
                    V[6]=V[1]*V[5]-V[2]*V[4];
                    V[7]=V[2]*V[3]-V[0]*V[5];
                    V[8]=V[0]*V[4]-V[1]*V[3];
                }
                
                for(j=0;j<10;j++){
                    
                    MM[0]=V[3*i];
                    MM[1]=V[3*i+1];
                    MM[2]=V[3*i+2];
                    
                    V[3*i]=invmat[0]*MM[0]+invmat[1]*MM[1]+invmat[2]*MM[2];
                    V[3*i+1]=invmat[1]*MM[0]+invmat[3]*MM[1]+invmat[4]*MM[2];
                    V[3*i+2]=invmat[2]*MM[0]+invmat[4]*MM[1]+invmat[5]*MM[2];
                    
                    if(k){
                        if(i==1){
                            distcc=V[3]*V[0]+V[4]*V[1]+V[5]*V[2];
                            V[3]=V[3]-distcc*V[0];
                            V[4]=V[4]-distcc*V[1];
                            V[5]=V[5]-distcc*V[2];
                        }
                    }
                    
                    distcc=sqrt(pwr2(V[3*i])+pwr2(V[3*i+1])+pwr2(V[3*i+2]));
                    
                    V[3*i]=V[3*i]/distcc;
                    V[3*i+1]=V[3*i+1]/distcc;
                    V[3*i+2]=V[3*i+2]/distcc;
                    
                    if(j>2){
                        if((pwr2(V[3*i]-MM[0])+pwr2(V[3*i+1]-MM[1])+pwr2(V[3*i+2]-MM[2]))<1e-29){
                            break;
                        }
                    }
                    
                }
            }
            
            /* singular values & V[i] of Spx found, U[i] ... */
            
            SpxTSpx[0]=Spx[0]*Spx[0]+Spx[3]*Spx[3]+Spx[6]*Spx[6];
            SpxTSpx[1]=Spx[1]*Spx[1]+Spx[4]*Spx[4]+Spx[7]*Spx[7];
            SpxTSpx[2]=Spx[2]*Spx[2]+Spx[5]*Spx[5]+Spx[8]*Spx[8];
            SpxTSpx[3]=Spx[0]*Spx[1]+Spx[3]*Spx[4]+Spx[6]*Spx[7];
            SpxTSpx[4]=Spx[1]*Spx[2]+Spx[4]*Spx[5]+Spx[7]*Spx[8];
            SpxTSpx[5]=Spx[0]*Spx[2]+Spx[3]*Spx[5]+Spx[6]*Spx[8];
            
            for(i=0;i<3;i++){
                
                invmat[0]=SpxTSpx[2]*SpxTSpx[1]-SpxTSpx[1]*SIGMA[i]-SIGMA[i]*SpxTSpx[2]+SIGMA[i]*SIGMA[i]-SpxTSpx[4]*SpxTSpx[4];
                invmat[1]=SpxTSpx[4]*SpxTSpx[5]-SpxTSpx[3]*SpxTSpx[2]+SpxTSpx[3]*SIGMA[i];
                invmat[2]=SpxTSpx[3]*SpxTSpx[4]-SpxTSpx[5]*SpxTSpx[1]+SpxTSpx[5]*SIGMA[i];
                invmat[3]=SpxTSpx[2]*SpxTSpx[0]-SpxTSpx[0]*SIGMA[i]-SIGMA[i]*SpxTSpx[2]+SIGMA[i]*SIGMA[i]-SpxTSpx[5]*SpxTSpx[5];
                invmat[4]=-SpxTSpx[4]*SpxTSpx[0]+SpxTSpx[4]*SIGMA[i]+SpxTSpx[3]*SpxTSpx[5];
                invmat[5]=SpxTSpx[1]*SpxTSpx[0]-SpxTSpx[0]*SIGMA[i]-SpxTSpx[1]*SIGMA[i]+SIGMA[i]*SIGMA[i]-SpxTSpx[3]*SpxTSpx[3];
                
                if(i<2){
                    U[3*i]=invmat[0];
                    U[3*i+1]=invmat[1];
                    U[3*i+2]=invmat[2];
                    
                    if(k){
                        if(i==1){
                            U[3]=invmat[1];
                            U[4]=invmat[3];
                            U[5]=invmat[4];
                            
                            distcc=U[3]*U[0]+U[4]*U[1]+U[5]*U[2];
                            U[3]=U[3]-distcc*U[0];
                            U[4]=U[4]-distcc*U[1];
                            U[5]=U[5]-distcc*U[2];
                        }
                    }
                    
                }
                else {
                    
                    /* Eigenvectors corresponding to symmetric positiv definite matrices
                     are orthogonal. */
                    
                    U[6]=U[1]*U[5]-U[2]*U[4];
                    U[7]=U[2]*U[3]-U[0]*U[5];
                    U[8]=U[0]*U[4]-U[1]*U[3];
                }
                
                for(j=0;j<10;j++){
                    
                    MM[0]=U[3*i];
                    MM[1]=U[3*i+1];
                    MM[2]=U[3*i+2];
                    
                    U[3*i]=invmat[0]*MM[0]+invmat[1]*MM[1]+invmat[2]*MM[2];
                    U[3*i+1]=invmat[1]*MM[0]+invmat[3]*MM[1]+invmat[4]*MM[2];
                    U[3*i+2]=invmat[2]*MM[0]+invmat[4]*MM[1]+invmat[5]*MM[2];
                    
                    if(k){
                        if(i==1){
                            distcc=U[3]*U[0]+U[4]*U[1]+U[5]*U[2];
                            U[3]=U[3]-distcc*U[0];
                            U[4]=U[4]-distcc*U[1];
                            U[5]=U[5]-distcc*U[2];
                        }
                    }
                    
                    distcc=sqrt(pwr2(U[3*i])+pwr2(U[3*i+1])+pwr2(U[3*i+2]));
                    
                    U[3*i]=U[3*i]/distcc;
                    U[3*i+1]=U[3*i+1]/distcc;
                    U[3*i+2]=U[3*i+2]/distcc;
                    
                    if(j>2){
                        if((pwr2(U[3*i]-MM[0])+pwr2(U[3*i+1]-MM[1])+pwr2(U[3*i+2]-MM[2]))<1e-29){
                            break;
                        }
                    }
                    
                }
                
            }
            
            k=0;
            for(i=0;i<3;i++){
                A=(Spx[0]*V[3*i]+Spx[3]*V[3*i+1]+Spx[6]*V[3*i+2])*U[3*i]+(Spx[1]*V[3*i]+Spx[4]*V[3*i+1]+Spx[7]*V[3*i+2])*U[3*i+1]+(Spx[2]*V[3*i]+Spx[5]*V[3*i+1]+Spx[8]*V[3*i+2])*U[3*i+2];
                if(A<0){
                    k++;
                    U[3*i]=-U[3*i];
                    U[3*i+1]=-U[3*i+1];
                    U[3*i+2]=-U[3*i+2];
                }
            }
               
      /* Get R=U*diag([1,1,det(U*V')])*V' */
            
            if(k==0 || k==2){           /* det(U*V')=+1 */
                for(i=0;i<3;i++){
                    for(j=0;j<3;j++){
                        R[i+3*j]=U[i]*V[j]+U[i+3]*V[j+3]+U[i+6]*V[j+6];
                    }
                }
            }
            else{                        /* det(U*V')=-1 */
                for(i=0;i<3;i++){
                    for(j=0;j<3;j++){
                        R[i+3*j]=U[i]*V[j]+U[i+3]*V[j+3]-U[i+6]*V[j+6];
                    }
                }
            }
            
    /* Get T=modelm-R*datam */
            
            T[0]=modelm[0]-R[0]*datam[0]-R[3]*datam[1]-R[6]*datam[2];
            T[1]=modelm[1]-R[1]*datam[0]-R[4]*datam[1]-R[7]*datam[2];
            T[2]=modelm[2]-R[2]*datam[0]-R[5]*datam[1]-R[8]*datam[2];
            
        }
        
    /* Get TR */
        
        for(j=0;j<3;j++){
            MM[0]=trpr[3*j];
            MM[1]=trpr[3*j+1];
            MM[2]=trpr[3*j+2];
            
            trpr[3*j]=R[0]*MM[0]+R[3]*MM[1]+R[6]*MM[2];
            trpr[1+3*j]=R[1]*MM[0]+R[4]*MM[1]+R[7]*MM[2];
            trpr[2+3*j]=R[2]*MM[0]+R[5]*MM[1]+R[8]*MM[2];
        }
        
    /* Get TT */
        
        MM[0]=ttpr[0];
        MM[1]=ttpr[1];
        MM[2]=ttpr[2];
        
        ttpr[0]=R[0]*MM[0]+R[3]*MM[1]+R[6]*MM[2]+T[0];
        ttpr[1]=R[1]*MM[0]+R[4]*MM[1]+R[7]*MM[2]+T[1];
        ttpr[2]=R[2]*MM[0]+R[5]*MM[1]+R[8]*MM[2]+T[2];

    }
    
}
//
//void mexFunction( int nlhs, mxArray *plhs[],
//int nrhs, const mxArray*prhs[] )
//{
//
//    if(nrhs != 7){
//        fprintf(stderr,("7 input arguments required.");
//    } else if (nlhs > 2) {
//        fprintf(stderr,("Too many output arguments.");
//    }
//    else if (mxGetM(model)!=3){
//        fprintf(stderr,("Dimension of model points must be 3.");
//    }
//    else if (mxGetM(data)!=3){
//        fprintf(stderr,("Dimension of data points must be 3.");
//    }
//    else if ((mxGetM(weights) != mxGetN(data)) && (mxGetN(weights) != mxGetN(data))){
//        fprintf(stderr,("Number of weights must be equal to the number of data points.");
//    }
//    else if ((mxGetM(randvec) != mxGetN(data)) && (mxGetN(randvec) != mxGetN(data))){
//        fprintf(stderr,("Length of randvec must be equal to the number of data points.");
//    }
//
//    TR = mxCreateDoubleMatrix(3,3, mxREAL);
//
//    TT = mxCreateDoubleMatrix(3,1, mxREAL);
//
//    //icp(mxGetPr(TR),mxGetPr(TT),mxGetPr(model),(unsigned int)mxGetN(model),mxGetPr(data),mxGetPr(weights),(unsigned int)mxGetN(data),mxGetPr(randvec),(unsigned int)mxGetN(randvec),(unsigned int)mxGetScalar(sizerand),(unsigned int)mxGetScalar(iter),mxGetPr(treeptr));
//    icp((double*)mxGetPr(TR),(double*)mxGetPr(TT),(double*)mxGetPr(model),(unsigned int)mxGetN(model),(double*)mxGetPr(data),(double*)mxGetPr(weights),(unsigned int)mxGetN(data),(unsigned int*)mxGetPr(randvec),(unsigned int)mxGetN(randvec),(unsigned int)mxGetScalar(sizerand),(unsigned int)mxGetScalar(iter),(double*)mxGetPr(treeptr));
//
//    return;
//
//}


}
