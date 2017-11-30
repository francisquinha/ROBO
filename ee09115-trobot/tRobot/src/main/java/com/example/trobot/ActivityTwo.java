package com.example.trobot;

import android.R.string;
import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.text.Editable;
import android.view.Menu;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import bsh.*;

public class ActivityTwo extends Activity {

	    
    public ActivityTwo() {
        //Log.i(TAG, "Instantiated new " + this.getClass());
    }

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.activity_two);
		
		final EditText code = (EditText)findViewById(R.id.editTextCode);
		final EditText console = (EditText)findViewById(R.id.editTextConsole);

		String stringCode = getIntent().getStringExtra("code");

		final double colorPositionX = getIntent().getDoubleExtra("colorPositionX", -1);
		final double colorPositionY = getIntent().getDoubleExtra("colorPositionY", -1);

		final boolean sensor1Active = getIntent().getBooleanExtra("sensor1Active", false);
		final boolean sensor2Active = getIntent().getBooleanExtra("sensor2Active", false);
		final boolean sensor3Active = getIntent().getBooleanExtra("sensor3Active", false);
		final boolean sensor4Active = getIntent().getBooleanExtra("sensor4Active", false);
		final boolean sensor5Active = getIntent().getBooleanExtra("sensor5Active", false);
		final boolean sensor6Active = getIntent().getBooleanExtra("sensor6Active", false);
		final boolean sensor7Active = getIntent().getBooleanExtra("sensor7Active", false);
		final boolean sensor8Active = getIntent().getBooleanExtra("sensor8Active", false);
		final boolean sensor9Active = getIntent().getBooleanExtra("sensor9Active", false);

		final double colorFillPercentage = getIntent().getDoubleExtra("colorFillPercentage", -1);
		
		if (stringCode!=null)
			code.append(stringCode);
		
		Button btnCheck = (Button)findViewById(R.id.btnCheck);
		btnCheck.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub
				String code_string = code.getText().toString();
				Interpreter interpreter = new Interpreter();
				
				try {
					interpreter.set("robotPosition", 160);
					interpreter.set("v1", 0);
					interpreter.set("v2", 0);
					interpreter.set("colorPositionX", colorPositionX);
					interpreter.set("colorPositionY", colorPositionY);
					interpreter.set("sensor1Active", sensor1Active);
					interpreter.set("sensor2Active", sensor2Active);
					interpreter.set("sensor3Active", sensor3Active);
					interpreter.set("sensor4Active", sensor4Active);
					interpreter.set("sensor5Active", sensor5Active);
					interpreter.set("sensor6Active", sensor6Active);
					interpreter.set("sensor7Active", sensor7Active);
					interpreter.set("sensor8Active", sensor8Active);
					interpreter.set("sensor9Active", sensor9Active);
					interpreter.set("colorFillPercentage", colorFillPercentage);
				} catch (EvalError e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				try {
					interpreter.eval(code_string);
					console.setText("Success: " + interpreter.get("v1").toString() + " " + interpreter.get("v2").toString());
					
				}catch (Exception e) {
					console.setText("Error: " + code_string);
				}
				
			}
		});
		
		Button btnSendCode = (Button)findViewById(R.id.btnCode);
		btnSendCode.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub
				String code_string = code.getText().toString();
				Intent intent = new Intent();
				intent.putExtra("code", code_string);
				setResult(RESULT_OK,intent);
				finish();
				//startActivity(intent);
				
			}
		});
		
		
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
	}
	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		//getMenuInflater().inflate(R.menu.main, menu);        
		return true;
	}
	
	@Override
	public void onBackPressed() {
		// TODO Auto-generated method stub
		super.onBackPressed();		
	}

}
