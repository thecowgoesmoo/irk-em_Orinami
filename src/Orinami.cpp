// Orinami: Rack module wrapper. The DSP lives in OrinamiCore.hpp.

#include "plugin.hpp"
#include "OrinamiCore.hpp"

using orinami::Controls;


struct Orinami : Module {
	enum ParamId {
		SEP_PARAM,
		ROT_PARAM,
		SHAPE_PARAM,
		FOLD_K_PARAM,
		FOLD_M_PARAM,
		FOLD_Z_PARAM,
		ROT_K_PARAM,
		ROT_Z_PARAM,
		COUPLE_PARAM,
		FOLD_KICK_AMT_PARAM,
		ROT_KICK_AMT_PARAM,
		FOLD_KICK_PARAM,
		ROT_KICK_PARAM,
		DC_PARAM,
		LEVEL_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		SEP_INPUT,
		ROT_INPUT,
		SHAPE_INPUT,
		FOLD_K_INPUT,
		FOLD_M_INPUT,
		FOLD_Z_INPUT,
		ROT_K_INPUT,
		ROT_Z_INPUT,
		AUDIO_INPUT,
		FOLD_KICK_INPUT,
		ROT_KICK_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		DEF_OUTPUT,
		ORTH_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		FOLD_LIGHT,
		ENUMS(ROT_LIGHT, 2),
		LIGHTS_LEN
	};

	orinami::Core core;

	int osIndex = 2;              // 0..3 -> 1x, 2x, 4x, 8x

	dsp::SchmittTrigger foldTrig, rotTrig;
	dsp::BooleanTrigger foldBtn, rotBtn;

	dsp::Upsampler<2, 8> up2;
	dsp::Upsampler<4, 8> up4;
	dsp::Upsampler<8, 8> up8;
	dsp::Decimator<2, 8> dnD2, dnO2;
	dsp::Decimator<4, 8> dnD4, dnO4;
	dsp::Decimator<8, 8> dnD8, dnO8;

	float dcX[2] = {}, dcY[2] = {};

	Orinami() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		// Each main control is an attenuverter on its CV jack. With nothing
		// patched the jack reads +10 V, so the knob sets the value directly.
		configParam(SEP_PARAM, -1.f, 1.f, 0.125f, "Separation (curl)", " wraps", 0.f, 4.f);
		configParam(ROT_PARAM, -1.f, 1.f, 0.f, "Rotation", "°", 0.f, 180.f);
		configParam(SHAPE_PARAM, -1.f, 1.f, 0.f, "Shape (L1 | L2 | L∞)");
		configParam(FOLD_K_PARAM, -1.f, 1.f, -1.f / 9.f, "Fold stiffness");
		configParam(FOLD_M_PARAM, -1.f, 1.f, 0.f, "Fold mass");
		configParam(FOLD_Z_PARAM, -1.f, 1.f, 0.158f, "Fold damping (left of noon self-oscillates)");
		configParam(ROT_K_PARAM, -1.f, 1.f, -1.f / 3.f, "Rotation stiffness");
		configParam(ROT_Z_PARAM, -1.f, 1.f, 0.158f, "Rotation damping (left of noon self-oscillates)");

		configParam(COUPLE_PARAM, 0.f, 1.f, 1.f, "Spin-fold coupling", "%", 0.f, 100.f);
		configParam(FOLD_KICK_AMT_PARAM, -1.f, 1.f, 0.15f, "Fold kick amount", "", 0.f, 2.f);
		configParam(ROT_KICK_AMT_PARAM, -1.f, 1.f, 1.f / 6.f, "Rotation kick amount", "°", 0.f, 180.f);
		configButton(FOLD_KICK_PARAM, "Kick fold");
		configButton(ROT_KICK_PARAM, "Kick rotation");
		configSwitch(DC_PARAM, 0.f, 1.f, 0.f, "DC block", {"Off", "On"});
		configSwitch(LEVEL_PARAM, 0.f, 1.f, 0.f, "Level compensation", {"Off", "On"});

		configInput(SEP_INPUT, "Separation CV");
		configInput(ROT_INPUT, "Rotation CV");
		configInput(SHAPE_INPUT, "Shape CV");
		configInput(FOLD_K_INPUT, "Fold stiffness CV");
		configInput(FOLD_M_INPUT, "Fold mass CV");
		configInput(FOLD_Z_INPUT, "Fold damping CV");
		configInput(ROT_K_INPUT, "Rotation stiffness CV");
		configInput(ROT_Z_INPUT, "Rotation damping CV");
		configInput(AUDIO_INPUT, "Signal");
		configInput(FOLD_KICK_INPUT, "Fold kick trigger");
		configInput(ROT_KICK_INPUT, "Rotation kick trigger");

		configOutput(DEF_OUTPUT, "Default projection");
		configOutput(ORTH_OUTPUT, "Orthogonal projection");

		configBypass(AUDIO_INPUT, DEF_OUTPUT);

		orinami::screenTables();   // build the tables off the audio thread
	}

	void onReset() override {
		core.reset();
		osIndex = 2;
		dcX[0] = dcX[1] = dcY[0] = dcY[1] = 0.f;
	}

	// knob x CV/10V, with the jack normalled to +10 V
	inline float control(int param, int input) {
		float cv = inputs[input].isConnected() ? inputs[input].getVoltage() : 10.f;
		return clamp(params[param].getValue() * cv * 0.1f, -1.f, 1.f);
	}

	static inline double dampingRatio(float c) {
		// right of noon: 0..2, squared for fine control near zero
		// left of noon: 0..-0.2
		return (c >= 0.f) ? 2.0 * c * c : 0.2 * c;
	}

	template <int OS, class TUp, class TDn>
	void shape(TUp* up, TDn* dnD, TDn* dnO, float in, float shapeIdx, bool levelComp, float& outD, float& outO) {
		float xs[OS];
		float bufD[OS], bufO[OS];
		if (OS > 1)
			up->process(in, xs);
		else
			xs[0] = in;
		for (int k = 0; k < OS; k++) {
			core.project(xs[k], (k + 1) / (float) OS, shapeIdx, levelComp, bufD[k], bufO[k]);
		}
		if (OS > 1) {
			outD = dnD->process(bufD);
			outO = dnO->process(bufO);
		}
		else {
			outD = bufD[0];
			outO = bufO[0];
		}
	}

	void process(const ProcessArgs& args) override {
		const double dt = args.sampleTime;

		// ---- controls ----------------------------------------------------
		Controls c;
		c.sep0 = 1.0 - 4.0 * std::fabs(control(SEP_PARAM, SEP_INPUT));               // 1 flat .. -3
		c.th0 = M_PI * control(ROT_PARAM, ROT_INPUT);                                // +/- 180 deg
		c.shapeIdx = orinami::shapeIndex(control(SHAPE_PARAM, SHAPE_INPUT));
		c.kf = std::pow(10.0, 3.5 + 4.5 * control(FOLD_K_PARAM, FOLD_K_INPUT));      // 1e-1 .. 1e8
		c.m = std::pow(10.0, 2.0 * control(FOLD_M_PARAM, FOLD_M_INPUT));             // 1e-2 .. 1e2
		c.zf = dampingRatio(control(FOLD_Z_PARAM, FOLD_Z_INPUT));
		c.kr = std::pow(10.0, 3.5 + 4.5 * control(ROT_K_PARAM, ROT_K_INPUT));
		c.zr = dampingRatio(control(ROT_Z_PARAM, ROT_Z_INPUT));
		c.couple = params[COUPLE_PARAM].getValue();
		float shapeIdx = c.shapeIdx;

		// ---- kicks: trigger jack or button -------------------------------
		bool fKick = foldTrig.process(inputs[FOLD_KICK_INPUT].getVoltage(), 0.1f, 1.f);
		fKick |= foldBtn.process(params[FOLD_KICK_PARAM].getValue() > 0.5f);
		bool rKick = rotTrig.process(inputs[ROT_KICK_INPUT].getVoltage(), 0.1f, 1.f);
		rKick |= rotBtn.process(params[ROT_KICK_PARAM].getValue() > 0.5f);
		double foldKick = fKick ? 2.0 * params[FOLD_KICK_AMT_PARAM].getValue() : 0.0;
		double rotKick = rKick ? M_PI * params[ROT_KICK_AMT_PARAM].getValue() : 0.0;

		core.step(c, dt, foldKick, rotKick);

		// ---- map the audio through the screen ----------------------------
		float in = inputs[AUDIO_INPUT].getVoltage() * 0.2f;      // +/-5 V -> +/-1
		bool levelComp = params[LEVEL_PARAM].getValue() > 0.5f;
		float oD = 0.f, oO = 0.f;
		switch (osIndex) {
			case 0: shape<1, dsp::Upsampler<2, 8>, dsp::Decimator<2, 8>>(nullptr, nullptr, nullptr, in, shapeIdx, levelComp, oD, oO); break;
			case 1: shape<2>(&up2, &dnD2, &dnO2, in, shapeIdx, levelComp, oD, oO); break;
			case 3: shape<8>(&up8, &dnD8, &dnO8, in, shapeIdx, levelComp, oD, oO); break;
			default: shape<4>(&up4, &dnD4, &dnO4, in, shapeIdx, levelComp, oD, oO); break;
		}

		// ---- optional DC block (10 Hz one-pole high-pass) ----------------
		if (params[DC_PARAM].getValue() > 0.5f) {
			float R = 1.f - (float) (2.0 * M_PI * 10.0 * dt);
			float o[2] = {oD, oO};
			for (int i = 0; i < 2; i++) {
				float yv = o[i] - dcX[i] + R * dcY[i];
				dcX[i] = o[i];
				dcY[i] = yv;
				o[i] = yv;
			}
			oD = o[0];
			oO = o[1];
		}
		else {
			dcX[0] = oD;
			dcX[1] = oO;
			dcY[0] = dcY[1] = 0.f;
		}

		outputs[DEF_OUTPUT].setVoltage(clamp(5.f * oD, -12.f, 12.f));
		outputs[ORTH_OUTPUT].setVoltage(clamp(5.f * oO, -12.f, 12.f));

		// ---- lights: curl amount, and rotation as green (+) / red (-) ----
		float rotN = std::sin((float) core.th);
		lights[FOLD_LIGHT].setBrightnessSmooth((float) (1.0 - core.u) * 0.25f, args.sampleTime);
		lights[ROT_LIGHT + 0].setBrightnessSmooth(std::max(rotN, 0.f), args.sampleTime);
		lights[ROT_LIGHT + 1].setBrightnessSmooth(std::max(-rotN, 0.f), args.sampleTime);
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "oversample", json_integer(osIndex));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* osJ = json_object_get(rootJ, "oversample");
		if (osJ)
			osIndex = clamp((int) json_integer_value(osJ), 0, 3);
	}
};


// Panel text is drawn with Rack's own UI font, since Rack's SVG renderer
// ignores <text> elements.
struct PanelText : Widget {
	struct Item {
		Vec pos;
		std::string text;
		float size;
		NVGcolor color;
		int align;
	};
	std::vector<Item> items;

	void add(Vec mm, const std::string& text, float size = 7.5f, NVGcolor color = nvgRGB(0x20, 0x20, 0x20), int align = NVG_ALIGN_CENTER) {
		items.push_back({mm2px(mm), text, size, color, align});
	}

	void draw(const DrawArgs& args) override {
		std::shared_ptr<window::Font> font = APP->window->loadFont(asset::system("res/fonts/Nunito-Bold.ttf"));
		if (!font || font->handle < 0)
			return;
		nvgFontFaceId(args.vg, font->handle);
		for (const Item& it : items) {
			nvgFontSize(args.vg, it.size);
			nvgTextLetterSpacing(args.vg, it.size > 12.f ? 2.f : 0.4f);
			nvgTextAlign(args.vg, it.align | NVG_ALIGN_MIDDLE);
			nvgFillColor(args.vg, it.color);
			nvgText(args.vg, it.pos.x, it.pos.y, it.text.c_str(), NULL);
		}
	}
};


struct OrinamiWidget : ModuleWidget {
	OrinamiWidget(Orinami* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Orinami.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		// Layout grid, mm. Keep in step with res/Orinami.svg.
		const float X[5] = {8.5f, 22.f, 35.5f, 49.f, 62.5f};
		const float KY[3] = {22.f, 52.f, 82.f};      // knob rows
		const float JY[3] = {34.f, 64.f, 94.f};      // CV jack rows
		const float BY = 114.f;                      // bottom row
		const float LBL = 7.3f;                      // label height above a control

		PanelText* txt = new PanelText;
		txt->box.size = box.size;
		addChild(txt);
		NVGcolor white = nvgRGB(0xff, 0xff, 0xff);
		NVGcolor dim = nvgRGB(0x60, 0x60, 0x60);

		txt->add(Vec(35.56f, 8.6f), "ORINAMI", 15.f);

		// Row 1: the screen
		addKnob(txt, X[0], KY[0], JY[0], LBL, "SEP", Orinami::SEP_PARAM, Orinami::SEP_INPUT);
		addKnob(txt, X[1], KY[0], JY[0], LBL, "ROT", Orinami::ROT_PARAM, Orinami::ROT_INPUT);
		addKnob(txt, X[2], KY[0], JY[0], LBL, "SHAPE", Orinami::SHAPE_PARAM, Orinami::SHAPE_INPUT);
		txt->add(Vec(X[3], KY[0] - LBL), "COUPLE");
		addParam(createParamCentered<Trimpot>(mm2px(Vec(X[3], KY[0])), module, Orinami::COUPLE_PARAM));
		txt->add(Vec(X[4], KY[0] - LBL), "FOLD");
		addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(X[4], KY[0])), module, Orinami::FOLD_LIGHT));
		txt->add(Vec(X[4], JY[0] - 5.5f), "SPIN");
		addChild(createLightCentered<MediumLight<GreenRedLight>>(mm2px(Vec(X[4], JY[0])), module, Orinami::ROT_LIGHT));

		// Row 2: fold dynamics
		txt->add(Vec(3.5f, KY[1] - 11.4f), "FOLD", 7.5f, dim, NVG_ALIGN_LEFT);
		addKnob(txt, X[0], KY[1], JY[1], LBL, "STIFF", Orinami::FOLD_K_PARAM, Orinami::FOLD_K_INPUT);
		addKnob(txt, X[1], KY[1], JY[1], LBL, "MASS", Orinami::FOLD_M_PARAM, Orinami::FOLD_M_INPUT);
		addKnob(txt, X[2], KY[1], JY[1], LBL, "DAMP", Orinami::FOLD_Z_PARAM, Orinami::FOLD_Z_INPUT);
		addKick(txt, X[3], X[4], KY[1], JY[1], LBL, Orinami::FOLD_KICK_AMT_PARAM, Orinami::FOLD_KICK_PARAM, Orinami::FOLD_KICK_INPUT);

		// Row 3: rotation dynamics (inertia comes from the fold mass)
		txt->add(Vec(3.5f, KY[2] - 11.4f), "ROTATION", 7.5f, dim, NVG_ALIGN_LEFT);
		addKnob(txt, X[0], KY[2], JY[2], LBL, "STIFF", Orinami::ROT_K_PARAM, Orinami::ROT_K_INPUT);
		addKnob(txt, X[2], KY[2], JY[2], LBL, "DAMP", Orinami::ROT_Z_PARAM, Orinami::ROT_Z_INPUT);
		addKick(txt, X[3], X[4], KY[2], JY[2], LBL, Orinami::ROT_KICK_AMT_PARAM, Orinami::ROT_KICK_PARAM, Orinami::ROT_KICK_INPUT);

		// Bottom row: signal in, post-processing switches, outputs
		txt->add(Vec(X[0], BY - LBL), "IN");
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(X[0], BY)), module, Orinami::AUDIO_INPUT));
		txt->add(Vec(X[1], BY - LBL), "DC BLK");
		addParam(createParamCentered<CKSS>(mm2px(Vec(X[1], BY)), module, Orinami::DC_PARAM));
		txt->add(Vec(X[2], BY - LBL), "LEVEL");
		addParam(createParamCentered<CKSS>(mm2px(Vec(X[2], BY)), module, Orinami::LEVEL_PARAM));
		txt->add(Vec(X[3], BY - LBL), "DEF", 7.5f, white);
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(X[3], BY)), module, Orinami::DEF_OUTPUT));
		txt->add(Vec(X[4], BY - LBL), "ORTH", 7.5f, white);
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(X[4], BY)), module, Orinami::ORTH_OUTPUT));
	}

	void addKnob(PanelText* txt, float x, float ky, float jy, float lbl, const char* name, int param, int input) {
		txt->add(Vec(x, ky - lbl), name);
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(x, ky)), module, param));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(x, jy)), module, input));
	}

	void addKick(PanelText* txt, float xa, float xb, float ky, float jy, float lbl, int amt, int btn, int trig) {
		txt->add(Vec(xa, ky - lbl), "AMOUNT");
		addParam(createParamCentered<Trimpot>(mm2px(Vec(xa, ky)), module, amt));
		txt->add(Vec(xb, ky - lbl), "KICK");
		addParam(createParamCentered<VCVButton>(mm2px(Vec(xb, ky)), module, btn));
		txt->add(Vec(xa, jy), "TRIG");
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(xb, jy)), module, trig));
	}

	void appendContextMenu(Menu* menu) override {
		Orinami* module = getModule<Orinami>();
		if (!module)
			return;
		menu->addChild(new MenuSeparator);
		menu->addChild(createIndexSubmenuItem("Oversampling", {"1x", "2x", "4x", "8x"},
			[=]() { return module->osIndex; },
			[=](int i) { module->osIndex = i; }));
	}
};


Model* modelOrinami = createModel<Orinami, OrinamiWidget>("Orinami");
