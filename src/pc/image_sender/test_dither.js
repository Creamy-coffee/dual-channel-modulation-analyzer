/* Tests the exact inline production algorithm without opening VISA or RF. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/index.html','utf8');
const source=html.slice(html.indexOf('function quantizeImage('),html.indexOf('\nlet img='));
const context={};vm.createContext(context);vm.runInContext(source,context);
const modes=['threshold','fs','atkinson','jjn','stucki','bayer4','bayer8','random'];
function image(n,gray,alpha=255){const a=new Uint8ClampedArray(n*n*4);for(let i=0;i<n*n;i++){a[i*4]=a[i*4+1]=a[i*4+2]=typeof gray==='function'?gray(i%n,Math.floor(i/n)):gray;a[i*4+3]=alpha;}return a;}
function run(a,n,mode,threshold=128,invert=false){return context.quantizeImage(a,n,n,mode,threshold,invert);}
let densities={};
for(const mode of modes){
 for(const n of [32,64,128]){
  for(const gray of [0,255]){const r=run(image(n,gray),n,mode);assert([...r.bits].every(b=>b===(gray===0?255:0)),mode+' must preserve solid endpoints');}
  const input=image(n,(x,y)=>Math.round(255*x/(n-1))),copy=Buffer.from(input),r=run(input,n,mode),again=run(input,n,mode),inv=run(input,n,mode,128,true);
  assert.deepStrictEqual(Buffer.from(input),copy,'input is not mutated');assert.deepStrictEqual(Buffer.from(r.bits),Buffer.from(again.bits),'stable preview on redraw');
  assert.strictEqual(r.bits.length,n*n/8);
  for(let i=0;i<n*n;i++){let bit=(r.bits[i>>3]>>(7-(i&7)))&1;assert.strictEqual(r.rgba[i*4],bit?0:255);assert.strictEqual(r.rgba[i*4+3],255);}
  for(let i=0;i<r.bits.length;i++)assert.strictEqual(inv.bits[i],r.bits[i]^255,'invert exact complement');
  const transparent=run(image(n,0,0),n,mode);assert([...transparent.bits].every(b=>b===0),'transparent composites to white');
 }
 let levels=[];
 for(const gray of [32,64,96,128,160,192,224]){const r=run(image(128,gray),128,mode);levels.push([...r.rgba].filter((v,i)=>i%4===0&&v===255).length/16384);}
 for(let i=1;i<levels.length;i++)assert(levels[i]>=levels[i-1]-.005,mode+' must preserve gray ordering');
 if(mode!=='threshold')for(let i=0;i<levels.length;i++)assert(Math.abs(levels[i]-[32,64,96,128,160,192,224][i]/255)<(mode==='atkinson'?.16:.04),mode+' gray density fidelity');
 densities[mode]=levels;
}
// Also compile the complete application script, not just the pure function.
new vm.Script(html.match(/<script>([\s\S]*?)<\/script>/)[1]);
console.log(JSON.stringify({pass:true,modes,densities,checks:'three sizes; endpoints; alpha; pack/preview equality; inversion; repeatability; gray ordering; full JS syntax'},null,2));
