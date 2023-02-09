#pragma once

const float SPEED_MAP[] = {0.000000000000000000e+00,
4.146589379673548681e-08,
3.317251820369321313e-07,
1.119561417552150662e-06,
2.653738470342198452e-06,
5.182990687135902164e-06,
8.956013052105471495e-06,
1.422145134916813491e-05,
2.122789232619770594e-05,
3.022385386158149556e-05,
4.145777513353543497e-05,
5.517800679248618945e-05,
7.163280113697823026e-05,
9.107030229348056505e-05,
1.137385364004123156e-04,
1.398854017969630714e-04,
1.697586592168121022e-04,
2.036059219874125636e-04,
2.416746462350211067e-04,
2.842121210960557587e-04,
3.314654589349208536e-04,
3.836815855688957555e-04,
4.411072305004759553e-04,
5.039889171573053162e-04,
5.725729531404488704e-04,
6.471054204810200439e-04,
7.278321659059117099e-04,
8.149987911127143381e-04,
9.088506430543485948e-04,
1.009632804233800973e-03,
1.117590083009295521e-03,
1.232967003910345749e-03,
1.356007797964936534e-03,
1.486956393038502133e-03,
1.626056404184728077e-03,
1.773551124008732049e-03,
1.929683513043123355e-03,
2.094696190137107505e-03,
2.268831422859080016e-03,
2.452331117913209013e-03,
2.645436811570284208e-03,
2.848389660113026522e-03,
3.061430430296774308e-03,
3.284799489825240837e-03,
3.518736797842259012e-03,
3.763481895439568792e-03,
4.019273896181452255e-03,
4.286351476645522407e-03,
4.564952866981997204e-03,
4.855315841489182827e-03,
5.157677709208496886e-03,
5.472275304536478036e-03,
5.799344977856779604e-03,
6.139122586190426389e-03,
6.491843483865999964e-03,
6.857742513209919011e-03,
7.237053995256204075e-03,
7.630011720477725135e-03,
8.036848939538043801e-03,
8.457798354064405277e-03,
8.893092107442990280e-03,
9.342961775635538757e-03,
9.807638358019288294e-03,
1.028735226824878390e-02,
1.078233332514172416e-02,
1.129281074358767789e-02,
1.181901312548089278e-02,
1.236116845067780634e-02,
1.291950406797826023e-02,
1.349424668613247169e-02,
1.408562236487304054e-02,
1.469385650597254678e-02,
1.531917384432712748e-02,
1.596179843906686546e-02,
1.662195366469237934e-02,
1.729986220223866944e-02,
1.799574603046666388e-02,
1.870982641708207606e-02,
1.944232390998290594e-02,
2.019345832853480793e-02,
2.096344875487610171e-02,
2.175251352525081616e-02,
2.256087022137254205e-02,
2.338873566181698394e-02,
2.423632589344537624e-02,
2.510385618285837506e-02,
2.599154100788048094e-02,
2.689959404907571461e-02,
2.782822818129548903e-02,
2.877765546525745677e-02,
2.974808713915677583e-02,
3.073973361031079321e-02,
3.175280444683503678e-02,
3.278750836935373503e-02,
3.384405324274331051e-02,
3.492264606790995707e-02,
3.602349297360074587e-02,
3.714679920825014747e-02,
3.829276913186097087e-02,
3.946160620791994145e-02,
4.065351299534969431e-02,
4.186869114049596163e-02,
4.310734136915106429e-02,
4.436966347861326376e-02,
4.565585632978386155e-02,
4.696611783930071393e-02,
4.830064497170871718e-02,
4.965963373166892847e-02,
5.104327915620487932e-02,
5.245177530698741375e-02,
5.388531526265838423e-02,
5.534409111119253932e-02,
5.682829394229926834e-02,
5.833811383986398091e-02,
5.987373987442790035e-02,
6.143536009570993439e-02,
6.302316152516751480e-02,
6.463733014859840420e-02,
6.627805090878358119e-02,
6.794550769817175873e-02,
6.963988335160486987e-02,
7.136135963908540880e-02,
7.311011725858629351e-02,
7.488633582890291684e-02,
7.669019388254805225e-02,
7.852186885868905897e-02,
8.038153709612949616e-02,
8.226937382633336959e-02,
8.418555316649301012e-02,
8.613024811264236025e-02,
8.810363053281278223e-02,
9.010587116023505150e-02,
9.213713958658531489e-02,
9.419760425527656800e-02,
9.628743245479542967e-02,
9.840679031208454663e-02,
1.005558427859716275e-01,
1.027347536606431078e-01,
1.049436855391663315e-01,
1.071827998370563639e-01,
1.094522567758919873e-01,
1.117522153769766602e-01,
1.140828334550485490e-01,
1.164442676120381881e-01,
1.188366732308728801e-01,
1.212602044693306125e-01,
1.237150142539422681e-01,
1.262012542739415766e-01,
1.287190749752652463e-01,
1.312686255546015035e-01,
1.338500539534881462e-01,
1.364635068524607808e-01,
1.391091296652499087e-01,
1.417870665330293045e-01,
1.444974603187139106e-01,
1.472404526013086912e-01,
1.500161836703078899e-01,
1.528247925201456914e-01,
1.556664168446980634e-01,
1.585411930318358920e-01,
1.614492561580298524e-01,
1.643907399830069060e-01,
1.673657769444594212e-01,
1.703744981528062530e-01,
1.734170333860063362e-01,
1.764935110844245703e-01,
1.796040583457519935e-01,
1.827488009199771390e-01,
1.859278632044124580e-01,
1.891413682387730333e-01,
1.923894377003095846e-01,
1.956721918989953179e-01,
1.989897497727666220e-01,
2.023422288828183868e-01,
2.057297454089533906e-01,
2.091524141449868646e-01,
2.126103484942052368e-01,
2.161036604648799431e-01,
2.196324606658373035e-01,
2.231968583020830232e-01,
2.267969611704818700e-01,
2.304328756554938629e-01,
2.341047067249659719e-01,
2.378125579259799949e-01,
2.415565313807556125e-01,
2.453367277826122850e-01,
2.491532463919847729e-01,
2.530061850324970552e-01,
2.568956400870929802e-01,
2.608217064942232044e-01,
2.647844777440898634e-01,
2.687840458749481964e-01,
2.728205014694661257e-01,
2.768939336511405669e-01,
2.810044300807724715e-01,
2.851520769529984900e-01,
2.893369589928821428e-01,
2.935591594525615022e-01,
2.978187601079566038e-01,
3.021158412555345896e-01,
3.064504817091328048e-01,
3.108227587968416250e-01,
3.152327483579457912e-01,
3.196805247399233663e-01,
3.241661607955059754e-01,
3.286897278797962230e-01,
3.332512958474452835e-01,
3.378509330498896679e-01,
3.424887063326471637e-01,
3.471646810326732835e-01,
3.518789209757754444e-01,
3.566314884740895419e-01,
3.614224443236144779e-01,
3.662518478018068624e-01,
3.711197566652368884e-01,
3.760262271473031603e-01,
3.809713139560081396e-01,
3.859550702717943205e-01,
3.909775477454408010e-01,
3.960387964960193630e-01,
4.011388651089126123e-01,
4.062778006338919612e-01,
4.114556485832555621e-01,
4.166724529300293023e-01,
4.219282561062266401e-01,
4.272230990011700591e-01,
4.325570209598739169e-01,
4.379300597814876905e-01,
4.433422517178011724e-01,
4.487936314718096176e-01,
4.542842321963422858e-01,
4.598140854927497134e-01,
4.653832214096547126e-01,
4.709916684417628785e-01,
4.766394535287361567e-01,
4.823266020541260302e-01,
4.880531378443708768e-01,
4.938190831678525017e-01,
4.996244587340155086e-01,
5.054692836925482879e-01,
5.113535756326251791e-01,
5.172773505822105822e-01,
5.232406230074253539e-01,
5.292434058119745988e-01,
5.352857103366366331e-01,
5.413675463588152326e-01,
5.474889220921518307e-01,
5.536498441862018893e-01,
5.598503177261701103e-01,
5.660903462327112612e-01,
5.723699316617893995e-01,
5.786890744046014889e-01,
5.850477732875609682e-01,
5.914460255723453797e-01,
5.978838269560031726e-01,
6.043611715711253440e-01,
6.108780519860766978e-01,
6.174344592052901648e-01,
6.240303826696226297e-01,
6.306658102567724855e-01,
6.373407282817591391e-01,
6.440551214974642447e-01,
6.508089730952352214e-01,
6.576022647055501658e-01,
6.644349763987440483e-01,
6.713070866857974162e-01,
6.782185725191863801e-01,
6.851694092937938851e-01,
6.921595708478840425e-01,
6.991890294641359693e-01,
7.062577558707407999e-01,
7.133657192425595372e-01,
7.205128872023421893e-01,
7.276992258220089660e-01,
7.349246996239912066e-01,
7.421892715826365894e-01,
7.494929031256724050e-01,
7.568355541357320027e-01,
7.642171829519414095e-01,
7.716377463715675677e-01,
7.790971996517266351e-01,
7.865954965111551234e-01,
7.941325891320398789e-01,
8.017084281619093478e-01,
8.093229627155867911e-01,
8.169761403772023423e-01,
8.246679072022663481e-01,
8.323982077198039953e-01,
8.401669849345493324e-01,
8.479741803291994673e-01,
8.558197338667298260e-01,
8.637035839927686975e-01,
8.716256676380328416e-01,
8.795859202208208272e-01,
8.875842756495694319e-01,
8.956206663254673295e-01,
9.036950231451290616e-01,
9.118072755033284071e-01,
9.199573512957923693e-01,
9.281451769220512293e-01,
9.363706772883526597e-01,
9.446337758106304605e-01,
9.529343944175346914e-01,
9.612724535535197568e-01,
9.696478721819918878e-01,
9.780605677885130245e-01,
9.865104563840676466e-01,
9.949974525083828958e-01,
1.003521469233310315e+00,
1.012082418166263764e+00,
1.020680209453716625e+00,
1.029314751784756288e+00,
1.037985952394694378e+00,
1.046693717068739149e+00,
1.055437950145720283e+00,
1.064218554521873861e+00,
1.073035431654683070e+00,
1.081888481566776861e+00,
1.090777602849883454e+00,
1.099702692668844239e+00,
1.108663646765678967e+00,
1.117660359463711783e+00,
1.126692723671750773e+00,
1.135760630888324130e+00,
1.144863971205972497e+00,
1.154002633315596604e+00,
1.163176504510859965e+00,
1.172385470692648424e+00,
1.181629416373580543e+00,
1.190908224682580396e+00,
1.200221777369497644e+00,
1.209569954809786019e+00,
1.218952636009234869e+00,
1.228369698608756444e+00,
1.237821018889225577e+00,
1.247306471776372883e+00,
1.256825930845733907e+00,
1.266379268327648244e+00,
1.275966355112316153e+00,
1.285587060754903588e+00,
1.295241253480703847e+00,
1.304928800190348070e+00,
1.314649566465073249e+00,
1.324403416572037306e+00,
1.334190213469689468e+00,
1.344009818813190593e+00,
1.353862092959885910e+00,
1.363746894974828816e+00,
1.373664082636355976e+00,
1.383613512441713933e+00,
1.393595039612734343e+00,
1.403608518101562286e+00,
1.413653800596432752e+00,
1.423730738527497319e+00,
1.433839182072703000e+00,
1.443978980163717951e+00,
1.454149980491906469e+00,
1.464352029514354836e+00,
1.474584972459945353e+00,
1.484848653335477664e+00,
1.495142914931840039e+00,
1.505467598830228404e+00,
1.515822545408412214e+00,
1.526207593847049626e+00,
1.536622582136047965e+00,
1.547067347080974375e+00,
1.557541724309507547e+00,
1.568045548277944512e+00,
1.578578652277746075e+00,
1.589140868442132648e+00,
1.599732027752723607e+00,
1.610351960046224606e+00,
1.621000494021157090e+00,
1.631677457244635976e+00,
1.642382676159189758e+00,
1.653115976089626793e+00,
1.663877181249943771e+00,
1.674666114750281043e+00,
1.685482598603918802e+00,
1.696326453734316342e+00,
1.707197499982199052e+00,
1.718095556112683164e+00,
1.729020439822445132e+00,
1.739971967746932968e+00,
1.750949955467620001e+00,
1.761954217519299704e+00,
1.772984567397422051e+00,
1.784040817565471837e+00,
1.795122779462386520e+00,
1.806230263510015366e+00,
1.817363079120618785e+00,
1.828521034704408077e+00,
1.839703937677120926e+00,
1.850911594467644417e+00,
1.862143810525670018e+00,
1.873400390329388099e+00,
1.884681137393224404e+00,
1.895985854275611171e+00,
1.907314342586797995e+00,
1.918666402996699105e+00,
1.930041835242779058e+00,
1.941440438137974400e+00,
1.952862009578651525e+00,
1.964306346552601390e+00,
1.975773245147072421e+00,
1.987262500556831402e+00,
1.998773907092270985e+00,
2.010307258187543411e+00,
2.021862346408732414e+00,
2.033438963462056215e+00,
2.045036900202111596e+00,
2.056655946640144172e+00,
2.068295891952354637e+00,
2.079956524488240088e+00,
2.091637631778965556e+00,
2.103339000545768833e+00,
2.115060416708397284e+00,
2.126801665393575735e+00,
2.138562530943506346e+00,
2.150342796924400446e+00,
2.162142246135040136e+00,
2.173960660615368301e+00,
2.185797821655112383e+00,
2.197653509802435234e+00,
2.209527504872616621e+00,
2.221419585956762255e+00,
2.233329531430542580e+00,
2.245257118962960874e+00,
2.257202125525146208e+00,
2.269164327399177505e+00,
2.281143500186931128e+00,
2.293139418818959996e+00,
2.305151857563395801e+00,
2.317180590034877419e+00,
2.329225389203507746e+00,
2.341286027403831849e+00,
2.353362276343846649e+00,
2.365453907114027920e+00,
2.377560690196387494e+00,
2.389682395473550436e+00,
2.401818792237861100e+00,
2.413969649200506051e+00,
2.426134734500665413e+00,
2.438313815714681532e+00,
2.450506659865258285e+00,
2.462713033430672382e+00,
2.474932702354013081e+00,
2.487165432052440561e+00,
2.499410987426464281e+00,
2.511669132869244869e+00,
2.523939632275912448e+00,
2.536222249052906719e+00,
2.548516746127337029e+00,
2.560822885956358430e+00,
2.573140430536569934e+00,
2.585469141413429650e+00,
2.597808779690683689e+00,
2.610159106039820820e+00,
2.622519880709538231e+00,
2.634890863535225058e+00,
2.647271813948461450e+00,
2.659662490986537176e+00,
2.672062653301982227e+00,
2.684472059172113401e+00,
2.696890466508598205e+00,
2.709317632867028536e+00,
2.721753315456513000e+00,
2.734197271149280350e+00,
2.746649256490298896e+00,
2.759109027706901252e+00,
2.771576340718434839e+00,
2.784050951145912833e+00,
2.796532614321681098e+00,
2.809021085299098441e+00,
2.821516118862223532e+00,
2.834017469535518252e+00,
2.846524891593555040e+00,
2.859038139070738005e+00,
2.871556965771034697e+00,
2.884081125277711877e+00,
2.896610370963085490e+00,
2.909144455998274204e+00,
2.921683133362965812e+00,
2.934226155855188090e+00,
2.946773276101086303e+00,
2.959324246564709604e+00,
2.971878819557802309e+00,
2.984436747249603616e+00,
2.996997781676647765e+00,
3.009561674752575744e+00,
3.022128178277947885e+00,
3.034697043950061790e+00,
3.047268023372774692e+00,
3.059840868066331154e+00,
3.072415329477187207e+00,
3.084991158987847815e+00,
3.097568107926699899e+00,
3.110145927577848024e+00,
3.122724369190953642e+00,
3.135303183991075660e+00,
3.147882123188509684e+00,
3.160460937988631702e+00,
3.173039379601737320e+00,
3.185617199252885445e+00,
3.198194148191737085e+00,
3.210769977702398137e+00,
3.223344439113254634e+00,
3.235917283806810207e+00,
3.248488263229523554e+00,
3.261057128901637459e+00,
3.273623632427009600e+00,
3.286187525502937579e+00,
3.298748559929981727e+00,
3.311306487621782590e+00,
3.323861060614875740e+00,
3.336412031078499041e+00,
3.348959151324397254e+00,
3.361502173816619532e+00,
3.374040851181311140e+00,
3.386574936216500298e+00,
3.399104181901873467e+00,
3.411628341408550202e+00,
3.424147168108846895e+00,
3.436660415586030304e+00,
3.449167837644066648e+00,
3.461669188317361368e+00,
3.474164221880486902e+00,
3.486652692857903801e+00,
3.499134356033672510e+00,
3.511608966461150061e+00,
3.524076279472684092e+00,
3.536536050689286448e+00,
3.548988036030304549e+00,
3.561431991723071899e+00,
3.573867674312556364e+00,
3.586294840670986694e+00,
3.598713248007471499e+00,
3.611122653877603117e+00,
3.623522816193048168e+00,
3.635913493231123894e+00,
3.648294443644360285e+00,
3.660665426470046668e+00,
3.673026201139764524e+00,
3.685376527488901655e+00,
3.697716165766155250e+00,
3.710044876643014966e+00,
3.722362421223226470e+00,
3.734668561052248315e+00,
3.746963058126678181e+00,
3.759245674903672452e+00,
3.771516174310340030e+00,
3.783774319753120619e+00,
3.796019875127144783e+00,
3.808252604825571819e+00,
3.820472273748912961e+00,
3.832678647314327058e+00,
3.844871491464903368e+00,
3.857050572678920819e+00,
3.869215657979079293e+00,
3.881366514941723800e+00,
3.893502911706034464e+00,
3.905624616983197850e+00,
3.917731400065556979e+00,
3.929823030835738251e+00,
3.941899279775753051e+00,
3.953959917976077598e+00,
3.966004717144707925e+00,
3.978033449616189543e+00,
3.990045888360625348e+00,
4.002041806992654216e+00,
4.014020979780408283e+00,
4.025983181654439136e+00,
4.037928188216624470e+00,
4.049855775749042763e+00,
4.061765721222823089e+00,
4.073657802306968279e+00,
4.085531797377149665e+00,
4.097387485524473405e+00,
4.109224646564216599e+00,
4.121043061044544764e+00,
4.132842510255184898e+00,
4.144622776236078998e+00,
4.156383641786009164e+00,
4.168124890471188060e+00,
4.179846306633816511e+00,
4.191547675400619788e+00,
4.203228782691344811e+00,
4.214889415227230707e+00,
4.226529360539441171e+00,
4.238148406977473748e+00,
4.249746343717529129e+00,
4.261322960770852930e+00,
4.272878048992041045e+00,
4.284411400087313915e+00,
4.295922806622753498e+00,
4.307412062032512701e+00,
4.318878960626983954e+00,
4.330323297600934040e+00,
4.341744869041610499e+00,
4.353143471936806286e+00,
4.364518904182886239e+00,
4.375870964592786905e+00,
4.387199452903973729e+00,
4.398504169786360940e+00,
4.409784916850196801e+00,
4.421041496653915104e+00,
4.432273712711940483e+00,
4.443481369502464418e+00,
4.454664272475177711e+00,
4.465822228058966559e+00,
4.476955043669569534e+00,
4.488062527717199046e+00,
4.499144489614113951e+00,
4.510200739782163737e+00,
4.521231089660285640e+00,
4.532235351711965343e+00,
4.543213339432652376e+00,
4.554164867357139990e+00,
4.565089751066902402e+00,
4.575987807197385848e+00,
4.586858853445269446e+00,
4.597702708575667430e+00,
4.608519192429303857e+00,
4.619308125929641129e+00,
4.630069331089958773e+00,
4.640802631020395808e+00,
4.651507849934949590e+00,
4.662184813158428476e+00,
4.672833347133360959e+00,
4.683453279426861293e+00,
4.694044438737452474e+00,
4.704606654901839491e+00,
4.715139758901640832e+00,
4.725643582870077353e+00,
4.736117960098611412e+00,
4.746562725043537156e+00,
4.756977713332536162e+00,
4.767362761771172686e+00,
4.777717708349356940e+00,
4.788042392247744417e+00,
4.798336653844107680e+00,
4.808600334719640657e+00,
4.818833277665230952e+00,
4.829035326687679763e+00,
4.839206327015867615e+00,
4.849346125106882788e+00,
4.859454568652088469e+00,
4.869531506583153480e+00,
4.879576789078022614e+00,
4.889590267566850557e+00,
4.899571794737870967e+00,
4.909521224543229145e+00,
4.919438412204756084e+00,
4.929323214219699878e+00,
4.939175488366395861e+00,
4.948995093709895876e+00,
4.958781890607548704e+00,
4.968535740714512094e+00,
4.978256506989238162e+00,
4.987944053698881497e+00,
4.997598246424681534e+00,
5.007218952067268525e+00,
5.016806038851937544e+00,
5.026359376333850548e+00,
5.035878835403212683e+00,
5.045364288290358878e+00,
5.054815608570828900e+00,
5.064232671170351807e+00,
5.073615352369799325e+00,
5.082963529810088588e+00,
5.092277082497004947e+00,
5.101555890806005245e+00,
5.110799836486936698e+00,
5.120008802668725600e+00,
5.129182673863987851e+00,
5.138321335973612847e+00,
5.147424676291260326e+00,
5.156492583507835015e+00,
5.165524947715873338e+00,
5.174521660413906154e+00,
5.183482614510741548e+00,
5.192407704329701446e+00,
5.201296825612809371e+00,
5.210149875524901830e+00,
5.218966752657712149e+00,
5.227747357033864617e+00,
5.236491590110846417e+00,
5.245199354784890744e+00,
5.253870555394829722e+00,
5.262505097725868275e+00,
5.271102889013321580e+00,
5.279663837946274363e+00,
5.288187854671202892e+00,
5.296674850795518807e+00,
5.305124739391072097e+00,
5.313537434997594566e+00,
5.321912853626065143e+00,
5.330250912762051207e+00,
5.338551531368954883e+00,
5.346814629891233572e+00,
5.355040130257534337e+00,
5.363227955883793641e+00,
5.371378031676256271e+00,
5.379490284034456948e+00,
5.387564640854118458e+00,
5.395601031530016023e+00,
5.403599386958765294e+00,
5.411559639541552613e+00,
5.419481723186817312e+00,
5.427365573312855851e+00,
5.435211126850386876e+00,
5.443018322245036344e+00,
5.450787099459781793e+00,
5.458517399977319329e+00,
5.466209166802383557e+00,
5.473862344463998220e+00,
5.481476879017676218e+00,
5.489052718047545909e+00,
5.496589810668430331e+00,
5.504088107527859819e+00,
5.511547560808017998e+00,
5.518968124227644267e+00,
5.526349753043853674e+00,
5.533692404053913272e+00,
5.540996035596949199e+00,
5.548260607555594248e+00,
5.555486081357576822e+00,
5.562672419977243266e+00,
5.569819587937026029e+00,
5.576927551308845210e+00,
5.583996277715450596e+00,
5.591025736331701523e+00,
5.598015897885792569e+00,
5.604966734660399297e+00,
5.611878220493788483e+00,
5.618750330780841296e+00,
5.625583042474035622e+00,
5.632376334084350233e+00,
5.639130185682121876e+00,
5.645844578897825983e+00,
5.652519496922812969e+00,
5.659154924509962825e+00,
5.665750847974295290e+00,
5.672307255193509867e+00,
5.678824135608460111e+00,
5.685301480223582615e+00,
5.691739281607240741e+00,
5.698137533892024820e+00,
5.704496232774983966e+00,
5.710815375517796610e+00,
5.717094960946874416e+00,
5.723334989453415567e+00,
5.729535462993383454e+00,
5.735696385087433846e+00,
5.741817760820770111e+00,
5.747899596842948711e+00,
5.753941901367611855e+00,
5.759944684172160656e+00,
5.765907956597375872e+00,
5.771831731546960498e+00,
5.777716023487037944e+00,
5.783560848445570279e+00,
5.789366224011733841e+00,
5.795132169335214911e+00,
5.800858705125460091e+00,
5.806545853650849409e+00,
5.812193638737823242e+00,
5.817802085769931075e+00,
5.823371221686835852e+00,
5.828901074983243724e+00,
5.834391675707776059e+00,
5.839843055461784616e+00,
5.845255247398098319e+00,
5.850628286219712315e+00,
5.855962208178415729e+00,
5.861257051073359037e+00,
5.866512854249556597e+00,
5.871729658596330559e+00,
5.876907506545693494e+00,
5.882046442070673287e+00,
5.887146510683566092e+00,
5.892207759434144876e+00,
5.897230236907791578e+00,
5.902213993223577759e+00,
5.907159080032283072e+00,
5.912065550514348899e+00,
5.916933459377779592e+00,
5.921762862855970866e+00,
5.926553818705496468e+00,
5.931306386203810455e+00,
5.936020626146913060e+00,
5.940696600846938402e+00,
5.945334374129696009e+00,
5.949934011332140393e+00,
5.954495579299790009e+00,
5.959019146384080479e+00,
5.963504782439662755e+00,
5.967952558821639997e+00,
5.972362548382744052e+00,
5.976734825470453316e+00,
5.981069465924051087e+00,
5.985366547071629739e+00,
5.989626147727024730e+00,
5.993848348186704200e+00,
5.998033230226587520e+00,
6.002180877098814094e+00,
6.006291373528444666e+00,
6.010364805710119995e+00,
6.014401261304637814e+00,
6.018400829435496036e+00,
6.022363600685363139e+00,
6.026289667092493474e+00,
6.030179122147089288e+00,
6.034032060787601459e+00,
6.037848579396973392e+00,
6.041628775798830731e+00,
6.045372749253606237e+00,
6.049080600454619372e+00,
6.052752431524091925e+00,
6.056388346009104140e+00,
6.059988448877502876e+00,
6.063552846513749373e+00,
6.067081646714706622e+00,
6.070574958685381439e+00,
6.074032893034599034e+00,
6.077455561770633174e+00,
6.080843078296767956e+00,
6.084195557406819610e+00,
6.087513115280590803e+00,
6.090795869479276980e+00,
6.094043938940813199e+00,
6.097257443975173885e+00,
6.100436506259608649e+00,
6.103581248833833683e+00,
6.106691796095161884e+00,
6.109768273793579674e+00,
6.112810809026780312e+00,
6.115819530235127033e+00,
6.118794567196579770e+00,
6.121736051021556158e+00,
6.124644114147750784e+00,
6.127518890334887836e+00,
6.130360514659440874e+00,
6.133169123509278009e+00,
6.135944854578277763e+00,
6.138687846860872099e+00,
6.141398240646556594e+00,
6.144076177514336656e+00,
6.146721800327125784e+00,
6.149335253226098530e+00,
6.151916681624984840e+00,
6.154466232204320875e+00,
6.156984052905644766e+00,
6.159470292925643520e+00,
6.161925102710255508e+00,
6.164348633948713463e+00,
6.166741039567547489e+00,
6.169102473724537461e+00,
6.171433091802609461e+00,
6.173733050403694023e+00,
6.176002507342530201e+00,
6.178241621640419901e+00,
6.180450553518943124e+00,
6.182629464393614604e+00,
6.184778516867501352e+00,
6.186897874724790469e+00,
6.188987702924309886e+00,
6.191048167593001139e+00,
6.193079436019351292e+00,
6.195081676646773339e+00,
6.197055059066943983e+00,
6.198999754013093444e+00,
6.200915933353252640e+00,
6.202803770083456847e+00,
6.204663438320896951e+00,
6.206495113297037847e+00,
6.208298971350683537e+00,
6.210075189920999605e+00,
6.211823947540501045e+00,
6.213545423827981473e+00,
6.215239799481414806e+00,
6.216907256270802762e+00,
6.218547977030987717e+00,
6.220162145654418495e+00,
6.221749947083876187e+00,
6.223311567305158221e+00,
6.224847193339722473e+00,
6.226357013237286964e+00,
6.227841216068394026e+00,
6.229299991916928292e+00,
6.230733531872599151e+00,
6.232142028023381464e+00,
6.233525673447917193e+00,
6.234884662207877071e+00,
6.236219189340285851e+00,
6.237529450849802259e+00,
6.238815643700973190e+00,
6.240077965810435501e+00,
6.241316616039090270e+00,
6.242531794184237093e+00,
6.243723700971666624e+00,
6.244892538047725594e+00,
6.246038507971336529e+00,
6.247161814205985486e+00,
6.248262661111676053e+00,
6.249341253936843366e+00,
6.250397798810232608e+00,
6.251432502732751750e+00,
6.252445573569275439e+00,
6.253437220040429345e+00,
6.254407651714329219e+00,
6.255357078998290632e+00,
6.256285713130511184e+00,
6.257193766171705640e+00,
6.258081450996727746e+00,
6.258948981286140523e+00,
6.259796571517769692e+00,
6.260624436958213579e+00,
6.261432793654335249e+00,
6.262221858424710241e+00,
6.262991848851051202e+00,
6.263742983269603215e+00,
6.264475480762504489e+00,
6.265189561149119513e+00,
6.265885444977347341e+00,
6.266563353514894352e+00,
6.267223508740519478e+00,
6.267866133335258993e+00,
6.268491450673614018e+00,
6.269099684814713136e+00,
6.269691060493453705e+00,
6.270265803111608527e+00,
6.270824138728908537e+00,
6.271366294054105950e+00,
6.271892496435998332e+00,
6.272402973854444674e+00,
6.272897954911337948e+00,
6.273377668821566999e+00,
6.273842345403950915e+00,
6.274292215072143186e+00,
6.274727508825521660e+00,
6.275148458240048299e+00,
6.275555295459108507e+00,
6.275948253184330028e+00,
6.276327564666376091e+00,
6.276693463695720787e+00,
6.277046184593396028e+00,
6.277385962201729619e+00,
6.277713031875049587e+00,
6.278027629470377846e+00,
6.278329991338097216e+00,
6.278620354312604235e+00,
6.278898955702940654e+00,
6.279166033283405390e+00,
6.279421825284146941e+00,
6.279666570381744251e+00,
6.279900507689760936e+00,
6.280123876749289735e+00,
6.280336917519473872e+00,
6.280539870368015976e+00,
6.280732976061672801e+00,
6.280916475756727735e+00,
6.281090610989449097e+00,
6.281255623666543109e+00,
6.281411756055577555e+00,
6.281559250775401892e+00,
6.281698350786547813e+00,
6.281829299381621823e+00,
6.281952340175676497e+00,
6.282067717096577297e+00,
6.282175674375352514e+00,
6.282276456536532550e+00,
6.282370308388474101e+00,
6.282457475013680792e+00,
6.282538201759105156e+00,
6.282612734226446172e+00,
6.282681318262429038e+00,
6.282744199949085839e+00,
6.282801625594017558e+00,
6.282853841720651644e+00,
6.282901095058490704e+00,
6.282943632533351419e+00,
6.282981701257599028e+00,
6.283015548520369364e+00,
6.283045421777789130e+00,
6.283071568643186389e+00,
6.283094236877293071e+00,
6.283113674378449254e+00,
6.283130129172794121e+00,
6.283143849404453363e+00,
6.283155083325724810e+00,
6.283164079287260506e+00,
6.283171085728237237e+00,
6.283176351166534168e+00,
6.283180124188898930e+00,
6.283182653441116372e+00,
6.283184187618169325e+00,
6.283184975454404686e+00,
6.283185265713692402e+00,
6.283185307179586232e+00,
};