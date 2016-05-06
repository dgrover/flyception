#include <Rcpp.h>
using namespace Rcpp;

// [[Rcpp::export]]
SEXP sfeaturesC (SEXP obj, SEXP ref, int nx, int ny, int nz) {
  StringVector nm(4);
  int fsize(nx * ny), nobj, no_objects, i, im, x, y;
  double val;
  NumericVector data(obj), refd(ref);
  List res(nz);

  nm[0] = "m.pxs";
  nm[1] = "m.int";
  nm[2] = "m.x";
  nm[3] = "m.y";

  for ( im = 0; im < nz; im++ ) {
      nobj = 0;
      no_objects = 0;
    /* get nobj */
    for (i = 0; i < fsize; i++ )
      if ( data[i + im * fsize] > nobj ) nobj = floor( data[i + im * fsize] );
    if ( nobj < 1 ) {
      no_objects = 1;
      nobj = 1; /* if no objects, create a matrix for 1 and fill all 0 */
      }
    
    // Create a matrix with dim names
    NumericMatrix m(nobj, 4);
    IntegerVector rn(nobj);
    for ( int j = 0; j < nobj; j++ ) rn[j] += j + 1;
    List dimnms = List::create(rn, nm);
    m.attr("dimnames") = dimnms;

    if ( no_objects ) continue;
    /* moment calculations for M00, M10, M01 */
    for ( x = 0; x < nx; x++ )
      for ( y = 0; y < ny; y++ ) {
        i = floor( data[ im * fsize + x + y * nx ] );
        if ( i < 1 ) continue;
        i--;
        if ( refd.size() != 1 )
          val = refd[ x + y * nx ];
        else
          val = 1.0;
        m[i           ] += 1.0;       /* pxs - size in pixels */
        m[i +     nobj] += val;       /* int / M00 - mass     */
        m[i + 2 * nobj] += x * val;   /* M10 - x moment       */
        m[i + 3 * nobj] += y * val;   /* M01 - y moment       */
      }
    
    /* convert M10, M01 to xm and ym */
    for ( i = 0; i < nobj; i++ ) {
      if ( (val = m[i + nobj]) == 0.0 ) continue;
      m[i + 2 * nobj] /= val;
      m[i + 3 * nobj] /= val;
    }
    res[im] = m;
  }
  
  return res;
}