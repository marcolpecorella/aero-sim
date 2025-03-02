import { Component } from '@angular/core';
import {MapComponent} from './map/map.component';
import {SidebarComponent} from './layout/sidebar/sidebar.component';
import {NavbarComponent} from './layout/navbar/navbar.component';

@Component({
  selector: 'app-root',
  imports: [MapComponent, SidebarComponent, NavbarComponent],
  templateUrl: './app.component.html',
  styleUrl: './app.component.css'
})
export class AppComponent {
  title = 'aero-sim';
  isSidebarOpen = false;
}
